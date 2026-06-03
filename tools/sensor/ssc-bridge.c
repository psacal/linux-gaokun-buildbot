/*
 * ssc-bridge — Bridge Qualcomm SSC sensor data to Linux evdev
 *
 * Reads sensor data via libssc (GLib/GObject API) and exposes it:
 *   - Accelerometer + Gyroscope → single uinput evdev IMU device
 *   - Light sensor              → /run/ssc-bridge/light (text file)
 *
 * Fills the gap between Qualcomm's SLPI DSP sensor pipeline and
 * standard Linux userspace (iio-sensor-proxy, InputPlumber, GNOME).
 *
 * Incorporates architectural improvements from ssc-uinput
 * (https://github.com/gio3k/ssc-uinput) by Gianni S. <me@gio.blue>.
 *
 * Build: make
 * Usage: ssc-bridge [--accel] [--gyro] [--light] [--all] [--daemon]
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
 * iio-sensor-proxy hardcodes 1G = 256 (drv-input-accel.c:142,
 * scale = 1/256 * 9.81).  Match that convention:
 *   raw = (m_s² / 9.81) * 256 = m_s² * 26.10
 * Range: ±4G = ±1024 raw units.  */
#define ACCEL_1G            256
#define ACCEL_SCALE         (ACCEL_1G / 9.81f)   /* 26.10 units per m/s² */
#define ACCEL_RES           ((int)(ACCEL_1G / 9.81f))
#define ACCEL_BASE_RANGE    (4 * ACCEL_1G)        /* ±4G = ±1024 */

/* Gyroscope: libssc reports rad/s.  Convert to deg/s * 1024.
 * Range: ±2000 deg/s.  */
#define GYRO_DEG_PER_RAD    (180.0f / (float)G_PI)
#define GYRO_UNITS_PER_DEG  1024.0f
#define GYRO_SCALE          (GYRO_UNITS_PER_DEG * GYRO_DEG_PER_RAD)
#define GYRO_RES            585
#define GYRO_BASE_RANGE     2097152               /* ±2000 deg/s * 1024 */

/* ── Configuration defaults ─────────────────────────────────────── */

#define SSC_OPEN_TIMEOUT    30              /* seconds per sensor */
#define DEFAULT_DEVICE_NAME "ssc-imu"

/* ── Data types ─────────────────────────────────────────────────── */

typedef float matrix3x3[9];   /* row-major: [r0c0, r0c1, r0c2, r1c0, ...] */

typedef struct {
    const char  *env_matrix;    /* SSCB_xxx_MOUNT_MATRIX env var     */
    int          uinput_fd;     /* shared IMU fd (-1 if unavailable) */
    const char  *file_path;     /* fallback file (NULL if none)      */
    matrix3x3    mount;         /* parsed mount matrix               */
    gboolean     active;
} Sensor;

typedef struct {
    GCancellable *cancellable;
    guint         timeout_id;
} SensorOpen;

/* ── Globals ────────────────────────────────────────────────────── */

static GMainLoop *main_loop = NULL;
static float      imu_output_scale = 1.0f;

/* ── Forward declarations ───────────────────────────────────────── */

static SensorOpen sensor_open_begin(guint timeout_seconds);
static void       sensor_open_end(SensorOpen *so);
static void       parse_mount_matrix(const char *env, matrix3x3 out);
static void       apply_mount_matrix(const matrix3x3 m, float *x, float *y, float *z);
static int        uinput_create_imu(const char *name,
                                    int accel_range, int gyro_range);
static void       emit_abs(int fd, int code, int value);
static void       emit_syn(int fd);
static void       write_vec3_file(const char *path, float x, float y, float z);
static void       write_light_file(const char *path, float lux);
static gboolean   on_signal_exit(gpointer user_data);
static float      parse_output_scale(void);

/* ── Sensor open helpers (GCancellable + timeout) ───────────────── */

static gboolean cancel_timeout_cb(gpointer user_data)
{
    g_cancellable_cancel(G_CANCELLABLE(user_data));
    return G_SOURCE_REMOVE;
}

static SensorOpen sensor_open_begin(guint timeout_seconds)
{
    SensorOpen so;
    so.cancellable = g_cancellable_new();
    so.timeout_id = g_timeout_add_seconds(timeout_seconds,
                                          cancel_timeout_cb,
                                          so.cancellable);
    return so;
}

static void sensor_open_end(SensorOpen *so)
{
    /* Remove timeout source before unref — prevents use-after-free
     * if the cancellable was freed while the timeout was pending. */
    g_source_remove(so->timeout_id);
    g_object_unref(so->cancellable);
}

/* ── Configuration parsing ──────────────────────────────────────── */

static float parse_output_scale(void)
{
    const char *val = g_getenv("SSCB_IMU_OUTPUT_SCALE");
    if (!val || !*val) return 1.0f;

    float scale = g_ascii_strtod(val, NULL);
    if (scale <= 0.0f) {
        g_warning("SSCB_IMU_OUTPUT_SCALE must be > 0, using 1.0");
        return 1.0f;
    }
    return scale;
}

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
        int vx = (int)(x * ACCEL_SCALE * imu_output_scale);
        int vy = (int)(y * ACCEL_SCALE * imu_output_scale);
        int vz = (int)(z * ACCEL_SCALE * imu_output_scale);

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
        int vx = (int)(x * GYRO_SCALE * imu_output_scale);
        int vy = (int)(y * GYRO_SCALE * imu_output_scale);
        int vz = (int)(z * GYRO_SCALE * imu_output_scale);

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

/* ── uinput: single IMU device (accel + gyro axes together) ─────── */

static int uinput_create_imu(const char *name,
                             int accel_range, int gyro_range)
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

    /* Mark as accelerometer for iio-sensor-proxy.
     * Gyroscope axes (ABS_RX/RY/RZ) on the same device are visible
     * to InputPlumber for gaming / motion control use cases. */
    if (ioctl(fd, UI_SET_PROPBIT, INPUT_PROP_ACCELEROMETER) < 0) {
        g_warning("UI_SET_PROPBIT failed: %s", strerror(errno));
        close(fd); return -1;
    }

    /* ── Accelerometer axes: ABS_X, ABS_Y, ABS_Z ─────────────── */
    struct uinput_abs_setup abs_setup = {0};
    int accel_axes[] = {ABS_X, ABS_Y, ABS_Z};
    for (int i = 0; i < 3; i++) {
        abs_setup.code = accel_axes[i];
        abs_setup.absinfo.value      = 0;
        abs_setup.absinfo.minimum    = -accel_range;
        abs_setup.absinfo.maximum    =  accel_range;
        abs_setup.absinfo.fuzz       = 128;
        abs_setup.absinfo.flat       = 64;
        abs_setup.absinfo.resolution = ACCEL_RES;
        if (ioctl(fd, UI_ABS_SETUP, &abs_setup) < 0) {
            g_warning("UI_ABS_SETUP %d failed: %s",
                      accel_axes[i], strerror(errno));
            close(fd); return -1;
        }
    }

    /* ── Gyroscope axes: ABS_RX, ABS_RY, ABS_RZ ──────────────── */
    int gyro_axes[] = {ABS_RX, ABS_RY, ABS_RZ};
    for (int i = 0; i < 3; i++) {
        abs_setup.code = gyro_axes[i];
        abs_setup.absinfo.value      = 0;
        abs_setup.absinfo.minimum    = -gyro_range;
        abs_setup.absinfo.maximum    =  gyro_range;
        abs_setup.absinfo.fuzz       = 512;
        abs_setup.absinfo.flat       = 256;
        abs_setup.absinfo.resolution = GYRO_RES;
        if (ioctl(fd, UI_ABS_SETUP, &abs_setup) < 0) {
            g_warning("UI_ABS_SETUP %d failed: %s",
                      gyro_axes[i], strerror(errno));
            close(fd); return -1;
        }
    }

    /* Create the device */
    struct uinput_setup usetup = {0};
    strncpy(usetup.name, name, UINPUT_MAX_NAME_SIZE - 1);
    usetup.id.bustype = BUS_VIRTUAL;
    usetup.id.vendor  = 0;
    usetup.id.product = 0;
    usetup.id.version = 1;
    if (ioctl(fd, UI_DEV_SETUP, &usetup) < 0 ||
        ioctl(fd, UI_DEV_CREATE) < 0) {
        g_warning("UI_DEV_CREATE failed: %s", strerror(errno));
        close(fd); return -1;
    }

    g_info("Created uinput IMU device: %s (accel ±%d, gyro ±%d)",
           name, accel_range, gyro_range);
    return fd;
}

/* ── Event emission ─────────────────────────────────────────────── */

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
        g_warning("%s has %d values (need 9), using identity", env_name, n);
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
    gboolean daemonize  = FALSE;

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
        else if (!strcmp(argv[i], "--version") || !strcmp(argv[i], "-V")) {
            printf("ssc-bridge 2.0.0\n");
            return 0;
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            printf("ssc-bridge — Bridge Qualcomm SSC sensors to Linux evdev\n\n"
                   "Usage: ssc-bridge [OPTIONS]\n\n"
                   "Options:\n"
                   "  --accel, -a    Bridge accelerometer\n"
                   "  --gyro,  -g    Bridge gyroscope\n"
                   "  --light, -l    Bridge ambient light sensor\n"
                   "  --all          Bridge all sensors (default)\n"
                   "  --daemon, -d   Fork to background\n"
                   "  --version, -V  Print version\n"
                   "  --help,  -h    Show this help\n\n"
                   "Output:\n"
                   "  /dev/input/event*        → IMU device (accel + gyro)\n"
                   "  /run/ssc-bridge/light    → ambient light (lux)\n"
                   "  /run/ssc-bridge/accel    → accelerometer fallback file\n"
                   "  /run/ssc-bridge/gyro     → gyroscope fallback file\n\n"
                   "Environment:\n"
                   "  SSCB_DEVICE_NAME                   uinput device name\n"
                   "                                     (default: ssc-imu)\n"
                   "  SSCB_IMU_OUTPUT_SCALE              scale factor for IMU\n"
                   "                                     (default: 1.0,\n"
                   "                                     use 100.0 for InputPlumber)\n"
                   "  SSCB_ACCELEROMETER_MOUNT_MATRIX    9 comma-sep floats\n"
                   "  SSCB_GYROSCOPE_MOUNT_MATRIX        9 comma-sep floats\n"
                   "  G_MESSAGES_DEBUG=all               Enable debug output\n");
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\nTry --help\n", argv[i]);
            return 1;
        }
    }

    /* Default: bridge all sensors */
    if (!want_accel && !want_gyro && !want_light)
        want_accel = want_gyro = want_light = TRUE;

    /* ── Parse configuration ──────────────────────────────────── */
    const char *dev_name = g_getenv("SSCB_DEVICE_NAME");
    if (!dev_name || !*dev_name)
        dev_name = DEFAULT_DEVICE_NAME;
    imu_output_scale = parse_output_scale();
    g_info("device=%s output_scale=%.2f", dev_name, imu_output_scale);

    /* Compute uinput ranges (scaled) */
    int accel_range = (int)(ACCEL_BASE_RANGE * imu_output_scale);
    int gyro_range  = (int)(GYRO_BASE_RANGE  * imu_output_scale);

    /* ── Daemonize ────────────────────────────────────────────── */
    if (daemonize) {
        if (fork() > 0) _exit(0);
        setsid();
        umask(0);
        int null_fd = open("/dev/null", O_RDONLY);
        if (null_fd >= 0) {
            dup2(null_fd, STDIN_FILENO);
            close(null_fd);
        }
    }

    /* Prepare output directory */
    g_mkdir_with_parents("/run/ssc-bridge", 0755);

    /* Signal handlers */
    g_unix_signal_add(SIGINT,  on_signal_exit, NULL);
    g_unix_signal_add(SIGTERM, on_signal_exit, NULL);
    signal(SIGPIPE, SIG_IGN);

    main_loop = g_main_loop_new(NULL, FALSE);

    /* ── Create shared IMU uinput device ──────────────────────── */
    int imu_fd = -1;
    if (want_accel || want_gyro)
        imu_fd = uinput_create_imu(dev_name, accel_range, gyro_range);

    /* ── Accelerometer ────────────────────────────────────────── */
    Sensor accel = { .env_matrix = "SSCB_ACCELEROMETER_MOUNT_MATRIX",
                     .uinput_fd = imu_fd,
                     .file_path = NULL, .active = FALSE };

    if (want_accel) {
        SensorOpen so = sensor_open_begin(SSC_OPEN_TIMEOUT);
        GError *err = NULL;
        SSCSensorAccelerometer *as =
            ssc_sensor_accelerometer_new_sync(so.cancellable, &err);

        if (!as) {
            if (g_error_matches(err, G_IO_ERROR, G_IO_ERROR_CANCELLED))
                g_warning("Accelerometer timed out after %ds — "
                          "is hexagonrpcd running?", SSC_OPEN_TIMEOUT);
            else
                g_warning("Accelerometer not available: %s", err->message);
            g_clear_error(&err);
        } else {
            parse_mount_matrix(accel.env_matrix, accel.mount);
            if (imu_fd < 0) {
                accel.file_path = "/run/ssc-bridge/accel";
                g_info("Accelerometer: file fallback → %s",
                       accel.file_path);
            }
            accel.active = TRUE;
            g_signal_connect(as, "measurement",
                             G_CALLBACK(accel_cb), &accel);

            if (!ssc_sensor_accelerometer_open_sync(as, so.cancellable, &err)) {
                if (g_error_matches(err, G_IO_ERROR, G_IO_ERROR_CANCELLED))
                    g_warning("Accelerometer open timed out");
                else
                    g_warning("Failed to open accelerometer: %s",
                              err->message);
                g_clear_error(&err);
                accel.active = FALSE;
            } else {
                g_info("Accelerometer bridge active (%s mode)",
                       imu_fd >= 0 ? "uinput" : "file");
            }
        }
        sensor_open_end(&so);
    }

    /* ── Gyroscope ────────────────────────────────────────────── */
    Sensor gyro = { .env_matrix = "SSCB_GYROSCOPE_MOUNT_MATRIX",
                    .uinput_fd = imu_fd,
                    .file_path = NULL, .active = FALSE };

    if (want_gyro) {
        SensorOpen so = sensor_open_begin(SSC_OPEN_TIMEOUT);
        GError *err = NULL;
        SSCSensorGyroscope *gs =
            ssc_sensor_gyroscope_new_sync(so.cancellable, &err);

        if (!gs) {
            if (g_error_matches(err, G_IO_ERROR, G_IO_ERROR_CANCELLED))
                g_warning("Gyroscope timed out after %ds — "
                          "is hexagonrpcd running?", SSC_OPEN_TIMEOUT);
            else
                g_warning("Gyroscope not available: %s", err->message);
            g_clear_error(&err);
        } else {
            parse_mount_matrix(gyro.env_matrix, gyro.mount);
            if (imu_fd < 0) {
                gyro.file_path = "/run/ssc-bridge/gyro";
                g_info("Gyroscope: file fallback → %s", gyro.file_path);
            }
            gyro.active = TRUE;
            g_signal_connect(gs, "measurement",
                             G_CALLBACK(gyro_cb), &gyro);

            if (!ssc_sensor_gyroscope_open_sync(gs, so.cancellable, &err)) {
                if (g_error_matches(err, G_IO_ERROR, G_IO_ERROR_CANCELLED))
                    g_warning("Gyroscope open timed out");
                else
                    g_warning("Failed to open gyroscope: %s",
                              err->message);
                g_clear_error(&err);
                gyro.active = FALSE;
            } else {
                g_info("Gyroscope bridge active (%s mode)",
                       imu_fd >= 0 ? "uinput" : "file");
            }
        }
        sensor_open_end(&so);
    }

    /* ── Light sensor ─────────────────────────────────────────── */
    if (want_light) {
        SensorOpen so = sensor_open_begin(SSC_OPEN_TIMEOUT);
        GError *err = NULL;
        SSCSensorLight *ls =
            ssc_sensor_light_new_sync(so.cancellable, &err);

        if (!ls) {
            if (g_error_matches(err, G_IO_ERROR, G_IO_ERROR_CANCELLED))
                g_warning("Light sensor timed out after %ds",
                          SSC_OPEN_TIMEOUT);
            else
                g_warning("Light sensor not available: %s", err->message);
            g_clear_error(&err);
        } else {
            g_signal_connect(ls, "measurement", G_CALLBACK(light_cb),
                             (gpointer)"/run/ssc-bridge/light");

            if (!ssc_sensor_light_open_sync(ls, so.cancellable, &err)) {
                if (g_error_matches(err, G_IO_ERROR, G_IO_ERROR_CANCELLED))
                    g_warning("Light sensor open timed out");
                else
                    g_warning("Failed to open light sensor: %s",
                              err->message);
                g_clear_error(&err);
            } else {
                g_info("Light sensor bridge active → "
                       "/run/ssc-bridge/light");
            }
        }
        sensor_open_end(&so);
    }

    /* ── Nothing bridged? ─────────────────────────────────────── */
    if (!accel.active && !gyro.active && !want_light) {
        g_warning("No sensors were successfully bridged");
        if (imu_fd >= 0) close(imu_fd);
        return 1;
    }

    g_info("ssc-bridge running, press Ctrl+C to stop");
    g_main_loop_run(main_loop);

    /* Cleanup */
    if (imu_fd >= 0) {
        ioctl(imu_fd, UI_DEV_DESTROY);
        close(imu_fd);
    }

    g_main_loop_unref(main_loop);
    return 0;
}
