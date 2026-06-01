/*
 * ssc-bridge — Bridge Qualcomm SSC sensor data to Linux evdev/IIO
 *
 * Reads sensor data via libssc (GLib/GObject API) and exposes it:
 *   - Accelerometer → uinput evdev (INPUT_PROP_ACCELEROMETER | ABS_X/Y/Z)
 *   - Gyroscope     → uinput evdev (ABS_RX/RY/RZ)
 *   - Light sensor  → /run/ssc-bridge/light (text file, lux value)
 *
 * This fills the gap between Qualcomm's SLPI DSP sensor pipeline and
 * standard Linux userspace (iio-sensor-proxy, GNOME auto-rotation).
 *
 * Build: make
 * Usage: ssc-bridge [--accel] [--gyro] [--light] [--all]
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <glib.h>
#include <glib-unix.h>
#include <libssc/libssc.h>

/* ── Scaling constants ──────────────────────────────────────────── */

/* Accelerometer: libssc reports m/s².  Scale to s32 for evdev.
 * Range: ±4G ≈ ±39.2 m/s².  Resolution ≈ units per m/s².  */
#define ACCEL_SCALE     1024.0f
#define ACCEL_RES       1024    /* units per m/s² — iio-sensor-proxy divides by this */
#define ACCEL_RANGE     40141   /* ±4G * 9.81 * 1024, capped for s32 sanity */

/* Gyroscope: libssc reports rad/s.  Convert to device units.
 * iio-sensor-proxy expects deg/s for in_anglvel.  */
#define GYRO_SCALE      (1024.0f * 180.0f / (float)G_PI)  /* units per rad/s */
#define GYRO_RES        585     /* ≈ 180/π * 1024 / 1024 … units per deg/s */
#define GYRO_RANGE      2097152 /* ±2000 deg/s * 1024 */

/* Light sensor: just write raw lux to a file.  */

/* ── Data types ─────────────────────────────────────────────────── */

typedef float matrix3x3[9];   /* row-major: [r0c0, r0c1, r0c2, r1c0, ...] */

typedef struct {
    const char  *name;          /* evdev device name            */
    const char  *env_matrix;    /* SSCB_xxx_MOUNT_MATRIX env var */
    int          uinput_fd;     /* /dev/uinput fd (-1 if unavailable) */
    const char  *file_path;     /* fallback file output (NULL if none) */
    matrix3x3    mount;         /* parsed mount matrix           */
    gboolean     active;
} Sensor;

/* ── Forward declarations ──────────────────────────────────────── */

static void parse_mount_matrix(const char *env_name, matrix3x3 out);
static void apply_mount_matrix(const matrix3x3 m, float *x, float *y, float *z);
static int  uinput_create_accel(void);
static int  uinput_create_gyro(void);
static void emit_abs(int fd, int code, int value);
static void emit_syn(int fd);
static void write_light_file(const char *path, float lux);
static gboolean on_signal_exit(gpointer user_data);

/* ── Sensor callbacks ───────────────────────────────────────────── */

static void write_vec3_file(const char *path, float x, float y, float z)
{
    char tmp[256];
    int len = snprintf(tmp, sizeof(tmp), "%.4f,%.4f,%.4f\n", x, y, z);
    if (len < 0 || len >= (int)sizeof(tmp)) return;

    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) return;
    if (write(fd, tmp, (size_t)len) != len) {}
    close(fd);
}

static void accel_cb(SSCSensorAccelerometer *sensor,
                     gfloat x, gfloat y, gfloat z, gpointer user_data)
{
    (void)sensor;
    Sensor *s = user_data;
    if (!s->active) return;

    apply_mount_matrix(s->mount, &x, &y, &z);

    if (s->uinput_fd >= 0) {
        /* Scale m/s² → device units, clamp to range */
        int vx = (int)(x * ACCEL_SCALE);
        int vy = (int)(y * ACCEL_SCALE);
        int vz = (int)(z * ACCEL_SCALE);

        if (vx < -ACCEL_RANGE) vx = -ACCEL_RANGE;
        if (vx >  ACCEL_RANGE) vx =  ACCEL_RANGE;
        if (vy < -ACCEL_RANGE) vy = -ACCEL_RANGE;
        if (vy >  ACCEL_RANGE) vy =  ACCEL_RANGE;
        if (vz < -ACCEL_RANGE) vz = -ACCEL_RANGE;
        if (vz >  ACCEL_RANGE) vz =  ACCEL_RANGE;

        emit_abs(s->uinput_fd, ABS_X, vx);
        emit_abs(s->uinput_fd, ABS_Y, vy);
        emit_abs(s->uinput_fd, ABS_Z, vz);
        emit_syn(s->uinput_fd);
    } else if (s->file_path) {
        write_vec3_file(s->file_path, x, y, z);
    }
}

static void gyro_cb(SSCSensorGyroscope *sensor,
                    gfloat x, gfloat y, gfloat z, gpointer user_data)
{
    (void)sensor;
    Sensor *s = user_data;
    if (!s->active) return;

    apply_mount_matrix(s->mount, &x, &y, &z);

    if (s->uinput_fd >= 0) {
        /* Scale rad/s → device units, clamp */
        int vx = (int)(x * GYRO_SCALE);
        int vy = (int)(y * GYRO_SCALE);
        int vz = (int)(z * GYRO_SCALE);

        if (vx < -GYRO_RANGE) vx = -GYRO_RANGE;
        if (vx >  GYRO_RANGE) vx =  GYRO_RANGE;
        if (vy < -GYRO_RANGE) vy = -GYRO_RANGE;
        if (vy >  GYRO_RANGE) vy =  GYRO_RANGE;
        if (vz < -GYRO_RANGE) vz = -GYRO_RANGE;
        if (vz >  GYRO_RANGE) vz =  GYRO_RANGE;

        emit_abs(s->uinput_fd, ABS_RX, vx);
        emit_abs(s->uinput_fd, ABS_RY, vy);
        emit_abs(s->uinput_fd, ABS_RZ, vz);
        emit_syn(s->uinput_fd);
    } else if (s->file_path) {
        write_vec3_file(s->file_path, x, y, z);
    }
}

static void light_cb(SSCSensorLight *sensor,
                     gfloat intensity, gpointer user_data)
{
    (void)sensor;
    const char *path = user_data;
    write_light_file(path, intensity);
}

/* ── uinput helpers ─────────────────────────────────────────────── */

static int uinput_create_accel(void)
{
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        g_warning("Cannot open /dev/uinput: %s", strerror(errno));
        return -1;
    }

    /* Enable event types */
    if (ioctl(fd, UI_SET_EVBIT, EV_ABS) < 0 ||
        ioctl(fd, UI_SET_EVBIT, EV_SYN) < 0) {
        g_warning("UI_SET_EVBIT failed: %s", strerror(errno));
        close(fd); return -1;
    }

    /* Mark as accelerometer for iio-sensor-proxy */
    if (ioctl(fd, UI_SET_PROPBIT, INPUT_PROP_ACCELEROMETER) < 0) {
        g_warning("UI_SET_PROPBIT failed: %s", strerror(errno));
        close(fd); return -1;
    }

    /* Set up ABS axes */
    struct uinput_abs_setup abs_setup = {0};
    int axes[] = {ABS_X, ABS_Y, ABS_Z};
    for (int i = 0; i < 3; i++) {
        abs_setup.code = axes[i];
        abs_setup.absinfo.value    = 0;
        abs_setup.absinfo.minimum  = -ACCEL_RANGE;
        abs_setup.absinfo.maximum  =  ACCEL_RANGE;
        abs_setup.absinfo.fuzz     = 128;   /* ~0.125 m/s² noise tolerance */
        abs_setup.absinfo.flat     = 64;    /* ~0.06 m/s² dead zone        */
        abs_setup.absinfo.resolution = ACCEL_RES;
        if (ioctl(fd, UI_ABS_SETUP, &abs_setup) < 0) {
            g_warning("UI_ABS_SETUP %d failed: %s", axes[i], strerror(errno));
            close(fd); return -1;
        }
    }

    /* Create device */
    struct uinput_setup usetup = {0};
    strncpy(usetup.name, "ssc-accelerometer", UINPUT_MAX_NAME_SIZE - 1);
    usetup.id.bustype = BUS_VIRTUAL;
    usetup.id.vendor  = 0;
    usetup.id.product = 0;
    usetup.id.version = 1;
    if (ioctl(fd, UI_DEV_SETUP, &usetup) < 0 ||
        ioctl(fd, UI_DEV_CREATE) < 0) {
        g_warning("UI_DEV_CREATE failed: %s", strerror(errno));
        close(fd); return -1;
    }

    g_info("Created uinput device: ssc-accelerometer");
    return fd;
}

static int uinput_create_gyro(void)
{
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        g_warning("Cannot open /dev/uinput: %s", strerror(errno));
        return -1;
    }

    if (ioctl(fd, UI_SET_EVBIT, EV_ABS) < 0 ||
        ioctl(fd, UI_SET_EVBIT, EV_SYN) < 0) {
        g_warning("UI_SET_EVBIT failed: %s", strerror(errno));
        close(fd); return -1;
    }

    struct uinput_abs_setup abs_setup = {0};
    int axes[] = {ABS_RX, ABS_RY, ABS_RZ};
    for (int i = 0; i < 3; i++) {
        abs_setup.code = axes[i];
        abs_setup.absinfo.value    = 0;
        abs_setup.absinfo.minimum  = -GYRO_RANGE;
        abs_setup.absinfo.maximum  =  GYRO_RANGE;
        abs_setup.absinfo.fuzz     = 512;
        abs_setup.absinfo.flat     = 256;
        abs_setup.absinfo.resolution = GYRO_RES;
        if (ioctl(fd, UI_ABS_SETUP, &abs_setup) < 0) {
            g_warning("UI_ABS_SETUP %d failed: %s", axes[i], strerror(errno));
            close(fd); return -1;
        }
    }

    struct uinput_setup usetup = {0};
    strncpy(usetup.name, "ssc-gyroscope", UINPUT_MAX_NAME_SIZE - 1);
    usetup.id.bustype = BUS_VIRTUAL;
    usetup.id.vendor  = 0;
    usetup.id.product = 0;
    usetup.id.version = 1;
    if (ioctl(fd, UI_DEV_SETUP, &usetup) < 0 ||
        ioctl(fd, UI_DEV_CREATE) < 0) {
        g_warning("UI_DEV_CREATE failed: %s", strerror(errno));
        close(fd); return -1;
    }

    g_info("Created uinput device: ssc-gyroscope");
    return fd;
}

static void emit_abs(int fd, int code, int value)
{
    struct input_event ev = {
        .type  = EV_ABS,
        .code  = (__u16)code,
        .value = (__s32)value,
    };
    if (write(fd, &ev, sizeof(ev)) != sizeof(ev))
        g_warning("write ABS_%d failed: %s", code, strerror(errno));
}

static void emit_syn(int fd)
{
    struct input_event ev = {
        .type  = EV_SYN,
        .code  = SYN_REPORT,
        .value = 0,
    };
    if (write(fd, &ev, sizeof(ev)) != sizeof(ev))
        g_warning("write SYN failed: %s", strerror(errno));
}

/* ── Mount matrix ───────────────────────────────────────────────── */

static void parse_mount_matrix(const char *env_name, matrix3x3 out)
{
    /* Default: identity */
    for (int i = 0; i < 9; i++)
        out[i] = 0.0f;
    out[0] = out[4] = out[8] = 1.0f;

    const char *val = g_getenv(env_name);
    if (!val || !*val) return;

    float v[9];
    int n = 0;
    char *copy = g_strdup(val);
    char *tok = strtok(copy, ", \t\n");
    while (tok && n < 9) {
        v[n++] = g_ascii_strtod(tok, NULL);
        tok = strtok(NULL, ", \t\n");
    }
    g_free(copy);

    if (n == 9) {
        memcpy(out, v, sizeof(v));
    } else {
        g_warning("%s has %d values (need 9), using identity matrix", env_name, n);
    }
}

static void apply_mount_matrix(const matrix3x3 m, float *x, float *y, float *z)
{
    float ix = *x, iy = *y, iz = *z;
    *x = m[0] * ix + m[1] * iy + m[2] * iz;
    *y = m[3] * ix + m[4] * iy + m[5] * iz;
    *z = m[6] * ix + m[7] * iy + m[8] * iz;
}

/* ── Light sensor file output ───────────────────────────────────── */

static void write_light_file(const char *path, float lux)
{
    char tmp[128];
    int len = snprintf(tmp, sizeof(tmp), "%.1f\n", lux);
    if (len < 0) return;

    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        g_warning("Cannot write %s: %s", path, strerror(errno));
        return;
    }
    if (write(fd, tmp, (size_t)len) != len)
        g_warning("write %s failed: %s", path, strerror(errno));
    close(fd);
}

/* ── Signal / cleanup ───────────────────────────────────────────── */

static GMainLoop *main_loop = NULL;

static gboolean on_signal_exit(gpointer user_data)
{
    (void)user_data;
    g_info("Received signal, shutting down...");
    if (main_loop)
        g_main_loop_quit(main_loop);
    return G_SOURCE_REMOVE;
}

/* ── Main ───────────────────────────────────────────────────────── */

int main(int argc, char **argv)
{
    gboolean want_accel = FALSE, want_gyro = FALSE, want_light = FALSE;
    gboolean daemonize = FALSE;

    /* Simple argument parsing — no GLib option context needed */
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--accel") || !strcmp(argv[i], "-a"))
            want_accel = TRUE;
        else if (!strcmp(argv[i], "--gyro") || !strcmp(argv[i], "-g"))
            want_gyro = TRUE;
        else if (!strcmp(argv[i], "--light") || !strcmp(argv[i], "-l"))
            want_light = TRUE;
        else if (!strcmp(argv[i], "--all"))
            want_accel = want_gyro = want_light = TRUE;
        else if (!strcmp(argv[i], "--daemon") || !strcmp(argv[i], "-d"))
            daemonize = TRUE;
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            printf("ssc-bridge — Bridge Qualcomm SSC sensors to Linux evdev\n\n"
                   "Usage: ssc-bridge [OPTIONS]\n\n"
                   "Options:\n"
                   "  --accel, -a    Bridge accelerometer\n"
                   "  --gyro,  -g    Bridge gyroscope\n"
                   "  --light, -l    Bridge ambient light sensor\n"
                   "  --all          Bridge all available sensors (default)\n"
                   "  --daemon, -d   Fork to background\n"
                   "  --help,  -h    Show this help\n\n"
                   "Output (uinput preferred, file fallback if /dev/uinput unavailable):\n"
                   "  /run/ssc-bridge/light    → ambient light (lux)\n"
                   "  /run/ssc-bridge/accel    → accelerometer (x,y,z m/s²)\n"
                   "  /run/ssc-bridge/gyro     → gyroscope (x,y,z rad/s)\n\n"
                   "Environment:\n"
                   "  SSCB_ACCELEROMETER_MOUNT_MATRIX   9 comma-separated floats\n"
                   "  SSCB_GYROSCOPE_MOUNT_MATRIX       9 comma-separated floats\n"
                   "  G_MESSAGES_DEBUG=all               Enable debug output\n");
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\nTry --help\n", argv[i]);
            return 1;
        }
    }

    /* Default: all sensors */
    if (!want_accel && !want_gyro && !want_light)
        want_accel = want_gyro = want_light = TRUE;

    /* Daemonize before creating GLib main context */
    if (daemonize) {
        if (fork() > 0) _exit(0);     /* Parent exits */
        setsid();
        umask(0);
        /* Close stdin, but keep stderr for GLib warnings */
        int null_fd = open("/dev/null", O_RDONLY);
        if (null_fd >= 0) {
            dup2(null_fd, STDIN_FILENO);
            close(null_fd);
        }
    }

    /* Prepare light sensor output directory */
    if (want_light)
        g_mkdir_with_parents("/run/ssc-bridge", 0755);

    /* Signal handlers */
    g_unix_signal_add(SIGINT,  on_signal_exit, NULL);
    g_unix_signal_add(SIGTERM, on_signal_exit, NULL);
    signal(SIGPIPE, SIG_IGN);

    main_loop = g_main_loop_new(NULL, FALSE);

    /* ── Accelerometer ─────────────────────────────────────────── */
    Sensor accel = { .name = "ssc-accelerometer",
                     .env_matrix = "SSCB_ACCELEROMETER_MOUNT_MATRIX",
                     .uinput_fd = -1, .file_path = NULL, .active = FALSE };

    if (want_accel) {
        GError *err = NULL;
        SSCSensorAccelerometer *as = ssc_sensor_accelerometer_new_sync(NULL, &err);
        if (!as) {
            g_warning("Accelerometer not available: %s", err->message);
            g_clear_error(&err);
        } else {
            parse_mount_matrix(accel.env_matrix, accel.mount);
            accel.uinput_fd = uinput_create_accel();
            if (accel.uinput_fd < 0) {
                /* uinput unavailable — fallback to file output */
                accel.file_path = "/run/ssc-bridge/accel";
                g_info("Accelerometer: file fallback → %s", accel.file_path);
            }
            accel.active = TRUE;
            g_signal_connect(as, "measurement", G_CALLBACK(accel_cb), &accel);
            if (!ssc_sensor_accelerometer_open_sync(as, NULL, &err)) {
                g_warning("Failed to open accelerometer: %s", err->message);
                g_clear_error(&err);
                accel.active = FALSE;
            } else {
                g_info("Accelerometer bridge active (%s mode)",
                       accel.uinput_fd >= 0 ? "uinput" : "file");
            }
        }
    }

    /* ── Gyroscope ─────────────────────────────────────────────── */
    Sensor gyro = { .name = "ssc-gyroscope",
                    .env_matrix = "SSCB_GYROSCOPE_MOUNT_MATRIX",
                    .uinput_fd = -1, .file_path = NULL, .active = FALSE };

    if (want_gyro) {
        GError *err = NULL;
        SSCSensorGyroscope *gs = ssc_sensor_gyroscope_new_sync(NULL, &err);
        if (!gs) {
            g_warning("Gyroscope not available: %s", err->message);
            g_clear_error(&err);
        } else {
            parse_mount_matrix(gyro.env_matrix, gyro.mount);
            gyro.uinput_fd = uinput_create_gyro();
            if (gyro.uinput_fd < 0) {
                /* uinput unavailable — fallback to file output */
                gyro.file_path = "/run/ssc-bridge/gyro";
                g_info("Gyroscope: file fallback → %s", gyro.file_path);
            }
            gyro.active = TRUE;
            g_signal_connect(gs, "measurement", G_CALLBACK(gyro_cb), &gyro);
            if (!ssc_sensor_gyroscope_open_sync(gs, NULL, &err)) {
                g_warning("Failed to open gyroscope: %s", err->message);
                g_clear_error(&err);
                gyro.active = FALSE;
            } else {
                g_info("Gyroscope bridge active (%s mode)",
                       gyro.uinput_fd >= 0 ? "uinput" : "file");
            }
        }
    }

    /* ── Light sensor ──────────────────────────────────────────── */
    if (want_light) {
        GError *err = NULL;
        SSCSensorLight *ls = ssc_sensor_light_new_sync(NULL, &err);
        if (!ls) {
            g_warning("Light sensor not available: %s", err->message);
            g_clear_error(&err);
        } else {
            g_signal_connect(ls, "measurement", G_CALLBACK(light_cb),
                             (gpointer)"/run/ssc-bridge/light");
            if (!ssc_sensor_light_open_sync(ls, NULL, &err)) {
                g_warning("Failed to open light sensor: %s", err->message);
                g_clear_error(&err);
            } else {
                g_info("Light sensor bridge active → /run/ssc-bridge/light");
            }
        }
    }

    /* ── Nothing bridged? ──────────────────────────────────────── */
    if (!accel.active && !gyro.active && !want_light) {
        g_warning("No sensors were successfully bridged");
        return 1;
    }

    g_info("ssc-bridge running, press Ctrl+C to stop");
    g_main_loop_run(main_loop);

    /* Cleanup */
    if (accel.uinput_fd >= 0) {
        ioctl(accel.uinput_fd, UI_DEV_DESTROY);
        close(accel.uinput_fd);
    }
    if (gyro.uinput_fd >= 0) {
        ioctl(gyro.uinput_fd, UI_DEV_DESTROY);
        close(gyro.uinput_fd);
    }

    g_main_loop_unref(main_loop);
    return 0;
}
