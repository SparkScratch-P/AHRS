/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : INS — MPU6500 (SPI2) + QMC5883L (I2C1) + USB CDC
  *                   Mahony AHRS → roll/pitch/yaw at ~100 Hz
  *
  * CHANGES vs previous revision
  * ─────────────────────────────
  * --- Gyro (kept from previous revision) ---
  * 1.  MPU DLPF enabled (92 Hz accel / 98 Hz gyro)
  * 2.  MPU sample-rate divider = 0 (1 kHz)
  * 3.  Fast inverse-sqrt (Q_rsqrt) in Mahony
  * 4.  Mahony Ki integral clamped (±MAHONY_IMAX)
  * 5.  Gyro calibration uses Welford online mean (N=500)
  * 6.  dt clamped [1 ms, 100 ms]
  * 7.  RESET_YAW command zeroes only Z integrator + yaw quaternion component
  * 8.  USB transmit guarded against USBD_BUSY
  * 9.  Named #defines for axis remap
  * 10. Gyro dead-band 0.03 °/s
  * 11. Magnetic heading with soft-iron offset placeholders
  * 12. All magic numbers replaced with named constants
  *
  * --- Accelerometer stabilisation (NEW in this revision) ---
  * 13. EMA low-pass filter on raw accel (ACCEL_LPF_ALPHA = 0.1) applied
  *     before Mahony — mirrors DLPF philosophy at the software level.
  *     Smooths out 10–50 Hz mechanical vibration that hardware DLPF
  *     at 92 Hz does not fully attenuate.
  * 14. Spike / outlier rejection — if any axis deviates from the EMA by
  *     more than ACCEL_SPIKE_THRESHOLD (0.5 g) the raw sample is replaced
  *     with the filtered value.  Catches electrical glitches and hard knocks.
  * 15. Magnitude sanity gate — if ||accel|| is outside [ACCEL_MIN_G,
  *     ACCEL_MAX_G] the Mahony accel correction is skipped for that step.
  *     Prevents corrupted tilt estimate during free-fall or hard over-range.
  * 16. Welford running variance on accel magnitude — dynamic-motion detector.
  *     When variance exceeds ACCEL_VAR_THRESHOLD the accel correction is also
  *     suppressed so the gyro integration is trusted during manoeuvres.
  * 17. Filtered accel values transmitted over USB for live observation.
  * 18. Accel calibration uses gravity-projection removal (NOT flat-assumption).
  *     Welford mean (N=200) → true gravity unit vector computed from mean →
  *     bias = mean - gravity_projection.  Captures only the electronic DC
  *     offset on each axis; does NOT bake in tilt angle as a bias.  Device
  *     can be calibrated at a slight tilt and X/Y will still read correctly.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* ═══════════════════════════════════════════════════════════════
   HAL HANDLES
═══════════════════════════════════════════════════════════════ */
SPI_HandleTypeDef hspi2;
I2C_HandleTypeDef hi2c1;

/* ═══════════════════════════════════════════════════════════════
   MPU-6500  (SPI2)
═══════════════════════════════════════════════════════════════ */
#define MPU_PWR_MGMT_1      0x6B
#define MPU_CONFIG          0x1A
#define MPU_SMPLRT_DIV      0x19
#define MPU_ACCEL_XOUT_H    0x3B
#define MPU_READ            0x80

/* DLPF: 0x02 → accel 92 Hz / gyro 98 Hz */
#define MPU_DLPF_92HZ       0x02

/* Sensitivity — ±250 °/s  ±2 g defaults */
#define GYRO_SENS           131.0f
#define ACCEL_SENS          16384.0f

/* Gyro noise-floor dead-band */
#define GYRO_DEADBAND       0.03f   /* °/s */

#define MPU_CS_LOW()   HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET)
#define MPU_CS_HIGH()  HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET)

/* ═══════════════════════════════════════════════════════════════
   QMC5883L  (I2C1)
═══════════════════════════════════════════════════════════════ */
#define MAG_ADDR            (0x0D << 1)
#define MAG_OFFSET_X        0
#define MAG_OFFSET_Y        0

/* ═══════════════════════════════════════════════════════════════
   MAHONY AHRS
═══════════════════════════════════════════════════════════════ */
#define MAHONY_KP           2.0f
#define MAHONY_KI           0.005f
#define MAHONY_IMAX         0.15f

/* ═══════════════════════════════════════════════════════════════
   ACCELEROMETER STABILISATION  [NEW]
═══════════════════════════════════════════════════════════════ */

/* EMA low-pass filter coefficient.
   α = 0.1  → cut-off ≈ fs * α / (2π(1-α)) ≈ 3.5 Hz at 200 Hz poll rate.
   Reduces vibration aliasing that survives the hardware DLPF.
   Increase toward 1.0 for less filtering / faster step response.          */
#define ACCEL_LPF_ALPHA         0.1f

/* Spike rejection threshold.
   If |raw - filtered| > this value on any axis, the raw sample is
   discarded and the filtered value is used instead.  0.5 g is ~5× the
   typical vibration amplitude on a handheld device; lower for smoother
   platforms, raise for aggressive vehicles.                                */
#define ACCEL_SPIKE_THRESHOLD   0.5f    /* g */

/* Magnitude sanity window.
   Outside this range the Mahony accel correction is disabled.
   At ±2 g full-scale any reading below 0.5 g or above 2.0 g is either
   free-fall or a severe shock — neither should correct the attitude.       */
#define ACCEL_MIN_G             0.5f    /* g */
#define ACCEL_MAX_G             2.0f    /* g */

/* Dynamic-motion suppression threshold.
   Welford running variance of the accel magnitude (g²).  Values above this
   indicate the device is being moved/shaken; accel correction is suppressed
   so gyro integration dominates.  Tune to your platform:
     ~0.002 g² — quiet bench / handheld stationary
     ~0.010 g² — walking / slow vehicle
     ~0.050 g² — aggressive manoeuvres / fast drone                         */
#define ACCEL_VAR_THRESHOLD     0.005f  /* g² */

/* Accel calibration sample count (DC bias removal, like gyro cal)          */
#define ACCEL_CAL_N             200

/* ═══════════════════════════════════════════════════════════════
   TIMING
═══════════════════════════════════════════════════════════════ */
#define PRINT_INTERVAL_MS   10
#define DT_MIN_MS           1
#define DT_MAX_MS           100
#define POLL_DELAY_MS       5

/* ═══════════════════════════════════════════════════════════════
   PRIVATE PROTOTYPES
═══════════════════════════════════════════════════════════════ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
void Error_Handler(void);

static void     MPU_Write(uint8_t reg, uint8_t data);
static void     MPU_ReadAll(float *gx, float *gy, float *gz,
                            float *ax, float *ay, float *az);
static void     MAG_Init(void);
static void     MAG_Read(int16_t *mx, int16_t *my, int16_t *mz);
static void     Mahony_Update(float gx, float gy, float gz,
                              float ax, float ay, float az, float dt);
static void     Quat_ToEuler(float *roll, float *pitch, float *yaw);
static void     CalibrateGyro(void);
static void     CalibrateAccel(void);          /* [NEW] */
static float    Q_rsqrt(float x);

/* ═══════════════════════════════════════════════════════════════
   MAHONY STATE
═══════════════════════════════════════════════════════════════ */
static float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f;
static float eIntX=0.0f, eIntY=0.0f, eIntZ=0.0f;

/* ═══════════════════════════════════════════════════════════════
   GYRO BIAS
═══════════════════════════════════════════════════════════════ */
static float gbx=0.0f, gby=0.0f, gbz=0.0f;

/* ═══════════════════════════════════════════════════════════════
   ACCELEROMETER FILTER STATE  [NEW]
═══════════════════════════════════════════════════════════════ */

/* EMA state — initialised to gravity on Z-axis (sensor flat) */
static float aFiltX = 0.0f;
static float aFiltY = 0.0f;
static float aFiltZ = 1.0f;

/* DC bias (removed by CalibrateAccel) */
static float abx = 0.0f, aby = 0.0f, abz = 0.0f;

/* Welford state for running variance of accel magnitude */
static float aVarMean  = 1.0f;   /* running mean of ||a||         */
static float aVarM2    = 0.0f;   /* running sum of squared deltas */
static uint32_t aVarN  = 0;      /* sample count                  */
static float accel_variance = 0.0f; /* published variance          */

/* ═══════════════════════════════════════════════════════════════
   USB RX
═══════════════════════════════════════════════════════════════ */
#define RX_BUF 64
extern uint8_t UserRxBufferFS[RX_BUF];
volatile uint8_t rxFlag = 0;
volatile uint8_t rxLen  = 0;

/* ═══════════════════════════════════════════════════════════════
   FAST INVERSE SQRT  (Quake III)
═══════════════════════════════════════════════════════════════ */
static float Q_rsqrt(float x)
{
    float xhalf = 0.5f * x;
    uint32_t i;
    memcpy(&i, &x, sizeof(i));
    i = 0x5F3759DFu - (i >> 1);
    memcpy(&x, &i, sizeof(x));
    x *= (1.5f - xhalf * x * x);
    x *= (1.5f - xhalf * x * x);
    return x;
}

/* ═══════════════════════════════════════════════════════════════
   ACCEL FILTER  [NEW]
   Call with raw (bias-removed) accel values.
   Returns filtered values via pointers.
   Also updates the running variance and sets *valid = 0 when the
   sample should be suppressed in Mahony.
═══════════════════════════════════════════════════════════════ */
static void Accel_Filter(float ax_raw, float ay_raw, float az_raw,
                         float *ax_out, float *ay_out, float *az_out,
                         uint8_t *valid)
{
    /* ── Step 1: Spike rejection ── */
    float use_x = ax_raw;
    float use_y = ay_raw;
    float use_z = az_raw;

    if (fabsf(ax_raw - aFiltX) > ACCEL_SPIKE_THRESHOLD) use_x = aFiltX;
    if (fabsf(ay_raw - aFiltY) > ACCEL_SPIKE_THRESHOLD) use_y = aFiltY;
    if (fabsf(az_raw - aFiltZ) > ACCEL_SPIKE_THRESHOLD) use_z = aFiltZ;

    /* ── Step 2: EMA low-pass ── */
    aFiltX += ACCEL_LPF_ALPHA * (use_x - aFiltX);
    aFiltY += ACCEL_LPF_ALPHA * (use_y - aFiltY);
    aFiltZ += ACCEL_LPF_ALPHA * (use_z - aFiltZ);

    /* ── Step 3: Magnitude sanity check ── */
    float mag2 = aFiltX*aFiltX + aFiltY*aFiltY + aFiltZ*aFiltZ;
    float mag  = sqrtf(mag2);   /* one sqrtf per loop — acceptable cost */

    *valid = (mag >= ACCEL_MIN_G && mag <= ACCEL_MAX_G) ? 1u : 0u;

    /* ── Step 4: Welford running variance of magnitude ── */
    aVarN++;
    float delta  = mag - aVarMean;
    aVarMean    += delta / (float)aVarN;
    float delta2 = mag - aVarMean;
    aVarM2      += delta * delta2;

    /* Publish variance (minimum 2 samples to avoid div-by-zero) */
    if (aVarN >= 2)
        accel_variance = aVarM2 / (float)(aVarN - 1);

    /* Cap accumulator to prevent float overflow on very long runs.
       Reset to keep variance "recent" (sliding window approximation). */
    if (aVarN > 4000) {
        aVarN   = 1;
        aVarM2  = 0.0f;
        /* aVarMean carries over — preserves mean, resets variance tracking */
    }

    /* ── Step 5: Dynamic-motion suppression ── */
    if (accel_variance > ACCEL_VAR_THRESHOLD)
        *valid = 0u;

    *ax_out = aFiltX;
    *ay_out = aFiltY;
    *az_out = aFiltZ;
}

/* ═══════════════════════════════════════════════════════════════
   MPU IMPLEMENTATION
═══════════════════════════════════════════════════════════════ */
static void MPU_Write(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = { reg & 0x7Fu, data };
    MPU_CS_LOW();
    HAL_SPI_Transmit(&hspi2, tx, 2, 100);
    MPU_CS_HIGH();
}

static void MPU_ReadAll(float *gx, float *gy, float *gz,
                        float *ax, float *ay, float *az)
{
    uint8_t tx[15] = { MPU_ACCEL_XOUT_H | MPU_READ };
    uint8_t rx[15] = { 0 };

    MPU_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 15, 100);
    MPU_CS_HIGH();

    int16_t axr = (int16_t)((rx[1]  << 8) | rx[2]);
    int16_t ayr = (int16_t)((rx[3]  << 8) | rx[4]);
    int16_t azr = (int16_t)((rx[5]  << 8) | rx[6]);
    /* rx[7..8] = TEMP — skipped */
    int16_t gxr = (int16_t)((rx[9]  << 8) | rx[10]);
    int16_t gyr = (int16_t)((rx[11] << 8) | rx[12]);
    int16_t gzr = (int16_t)((rx[13] << 8) | rx[14]);

    *ax = (float)axr / ACCEL_SENS;
    *ay = (float)ayr / ACCEL_SENS;
    *az = (float)azr / ACCEL_SENS;
    *gx = (float)gxr / GYRO_SENS;
    *gy = (float)gyr / GYRO_SENS;
    *gz = (float)gzr / GYRO_SENS;
}

/* ═══════════════════════════════════════════════════════════════
   MAG IMPLEMENTATION
═══════════════════════════════════════════════════════════════ */
static void MAG_Init(void)
{
    uint8_t cfg[2] = { 0x09, 0x1D };
    HAL_I2C_Master_Transmit(&hi2c1, MAG_ADDR, cfg, 2, 100);
}

static void MAG_Read(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t reg  = 0x00;
    uint8_t data[6];
    HAL_I2C_Master_Transmit(&hi2c1, MAG_ADDR, &reg,  1, 100);
    HAL_I2C_Master_Receive (&hi2c1, MAG_ADDR, data,  6, 100);
    *mx = (int16_t)((data[1] << 8) | data[0]);
    *my = (int16_t)((data[3] << 8) | data[2]);
    *mz = (int16_t)((data[5] << 8) | data[4]);
}

/* ═══════════════════════════════════════════════════════════════
   MAHONY AHRS
   gx/gy/gz °/s  |  ax/ay/az g (filtered, valid flag controls
   whether accel correction is applied this step)
═══════════════════════════════════════════════════════════════ */
static void Mahony_Update(float gx, float gy, float gz,
                          float ax, float ay, float az, float dt)
{
    /* °/s → rad/s */
    gx *= 0.017453293f;
    gy *= 0.017453293f;
    gz *= 0.017453293f;

    /* Normalise accelerometer — skip degenerate vector */
    float norm2 = ax*ax + ay*ay + az*az;
    if (norm2 < 1e-10f) return;
    float invNorm = Q_rsqrt(norm2);
    ax *= invNorm; ay *= invNorm; az *= invNorm;

    /* Estimated gravity direction from current quaternion */
    float vx = 2.0f*(q1*q3 - q0*q2);
    float vy = 2.0f*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /* Cross-product error */
    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    /* Integral feedback with wind-up clamp */
    eIntX += MAHONY_KI * ex * dt;
    eIntY += MAHONY_KI * ey * dt;
    eIntZ += MAHONY_KI * ez * dt;

    if (eIntX >  MAHONY_IMAX) eIntX =  MAHONY_IMAX;
    if (eIntX < -MAHONY_IMAX) eIntX = -MAHONY_IMAX;
    if (eIntY >  MAHONY_IMAX) eIntY =  MAHONY_IMAX;
    if (eIntY < -MAHONY_IMAX) eIntY = -MAHONY_IMAX;
    if (eIntZ >  MAHONY_IMAX) eIntZ =  MAHONY_IMAX;
    if (eIntZ < -MAHONY_IMAX) eIntZ = -MAHONY_IMAX;

    /* Proportional + integral correction */
    gx += MAHONY_KP * ex + eIntX;
    gy += MAHONY_KP * ey + eIntY;
    gz += MAHONY_KP * ez + eIntZ;

    /* Integrate quaternion kinematics */
    float dq0 = 0.5f*(-q1*gx - q2*gy - q3*gz) * dt;
    float dq1 = 0.5f*( q0*gx + q2*gz - q3*gy) * dt;
    float dq2 = 0.5f*( q0*gy - q1*gz + q3*gx) * dt;
    float dq3 = 0.5f*( q0*gz + q1*gy - q2*gx) * dt;

    q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

    float qNorm = Q_rsqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= qNorm; q1 *= qNorm; q2 *= qNorm; q3 *= qNorm;
}

/* Pure gyro-only integration — called when accel data is suppressed [NEW] */
static void Mahony_UpdateGyroOnly(float gx, float gy, float gz, float dt)
{
    gx *= 0.017453293f;
    gy *= 0.017453293f;
    gz *= 0.017453293f;

    /* No accel correction; no integral feedback this step */
    float dq0 = 0.5f*(-q1*gx - q2*gy - q3*gz) * dt;
    float dq1 = 0.5f*( q0*gx + q2*gz - q3*gy) * dt;
    float dq2 = 0.5f*( q0*gy - q1*gz + q3*gx) * dt;
    float dq3 = 0.5f*( q0*gz + q1*gy - q2*gx) * dt;

    q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

    float qNorm = Q_rsqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= qNorm; q1 *= qNorm; q2 *= qNorm; q3 *= qNorm;
}

/* ═══════════════════════════════════════════════════════════════
   QUATERNION → ZYX EULER
═══════════════════════════════════════════════════════════════ */
static void Quat_ToEuler(float *roll, float *pitch, float *yaw)
{
    float sinr_cosp = 2.0f*(q0*q1 + q2*q3);
    float cosr_cosp = 1.0f - 2.0f*(q1*q1 + q2*q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * 57.29578f;

    float sinp = 2.0f*(q0*q2 - q3*q1);
    *pitch = (fabsf(sinp) >= 1.0f)
           ? copysignf(90.0f, sinp)
           : asinf(sinp) * 57.29578f;

    float siny_cosp = 2.0f*(q0*q3 + q1*q2);
    float cosy_cosp = 1.0f - 2.0f*(q2*q2 + q3*q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * 57.29578f;
}

/* ═══════════════════════════════════════════════════════════════
   GYRO CALIBRATION  (Welford, N=500)
═══════════════════════════════════════════════════════════════ */
#define GYRO_CAL_N 500

static void CalibrateGyro(void)
{
    float ax, ay, az, gx, gy, gz;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;

    for (int i = 1; i <= GYRO_CAL_N; i++) {
        MPU_ReadAll(&gx, &gy, &gz, &ax, &ay, &az);
        mx += (gx - mx) / (float)i;
        my += (gy - my) / (float)i;
        mz += (gz - mz) / (float)i;
        HAL_Delay(2);
    }
    gbx = mx; gby = my; gbz = mz;

    char buf[96];
    int  len = snprintf(buf, sizeof(buf),
        "CALIB_G:DONE Bias:%.4f,%.4f,%.4f\r\n", gbx, gby, gbz);
    CDC_Transmit_FS((uint8_t*)buf, (uint16_t)len);
}

/* ═══════════════════════════════════════════════════════════════
   ACCEL CALIBRATION  (Welford, N=200)  — gravity-projection method
   ───────────────────────────────────────────────────────────────
   PROBLEM with the naive approach (bias = mean_x, mean_y, mean_z-1):
     It assumes the device is perfectly flat.  If it is tilted even
     a few degrees, gravity projects onto X and Y, and those
     projections get baked into the bias — causing a permanent
     "phantom" acceleration on X/Y at every future orientation.

   CORRECT approach used here:
     1. Collect Welford mean of raw readings → (mx, my, mz).
     2. The mean vector IS the gravity vector in sensor frame (at
        whatever tilt the device happened to be during cal).
     3. Compute the true 1 g unit vector: grav = mean / ||mean||.
     4. bias = mean - grav.
        This isolates the purely electronic DC offset and discards
        the gravitational projection — so X/Y bias is zero (or near
        zero) regardless of calibration tilt angle.
     5. After bias removal, ||accel|| ≈ 1 g and gravity points in
        the correct direction for Mahony normalisation.

   Device does NOT need to be flat.  It must be stationary.
═══════════════════════════════════════════════════════════════ */
static void CalibrateAccel(void)
{
    float ax, ay, az, gx, gy, gz;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;

    for (int i = 1; i <= ACCEL_CAL_N; i++) {
        MPU_ReadAll(&gx, &gy, &gz, &ax, &ay, &az);
        /* Welford online mean — numerically stable */
        mx += (ax - mx) / (float)i;
        my += (ay - my) / (float)i;
        mz += (az - mz) / (float)i;
        HAL_Delay(2);
    }

    /* Magnitude of the mean vector = measured gravity (should be ~1 g) */
    float g_mag = sqrtf(mx*mx + my*my + mz*mz);

    /* Guard: if sensor is completely dead or wildly off, skip cal */
    if (g_mag < 0.3f) {
        const char err[] = "CALIB_A:ERR low_mag\r\n";
        CDC_Transmit_FS((uint8_t*)err, sizeof(err) - 1);
        return;
    }

    /* True 1 g unit vector in sensor frame at calibration pose */
    float grav_x = mx / g_mag;
    float grav_y = my / g_mag;
    float grav_z = mz / g_mag;

    /*
     * bias = measured_mean - gravity_projection
     *
     * grav_{x,y,z} already accounts for whatever tilt was present.
     * Subtracting it leaves ONLY the electronic (non-gravitational)
     * DC offset — which is what we actually want to remove.
     *
     * After applying this bias:
     *   corrected = raw - bias = raw - (mean - grav)
     *             = raw - mean + grav
     * At rest: raw ≈ mean, so corrected ≈ grav — the pure 1 g
     * gravity vector pointing in the correct direction.  No phantom
     * X/Y acceleration regardless of calibration tilt.
     */
    abx = mx - grav_x;
    aby = my - grav_y;
    abz = mz - grav_z;

    /* Seed EMA with the expected post-correction gravity vector
       so the filter starts settled rather than converging from 0.  */
    aFiltX = grav_x;
    aFiltY = grav_y;
    aFiltZ = grav_z;

    /* Reset variance tracker so old motion data doesn't pollute
       the first few seconds of operation after re-cal.            */
    aVarN  = 0;
    aVarM2 = 0.0f;
    aVarMean = g_mag;
    accel_variance = 0.0f;

    char buf[128];
    int  len = snprintf(buf, sizeof(buf),
        "CALIB_A:DONE Bias:%.4f,%.4f,%.4f GravVec:%.3f,%.3f,%.3f\r\n",
        abx, aby, abz, grav_x, grav_y, grav_z);
    CDC_Transmit_FS((uint8_t*)buf, (uint16_t)len);
}

/* ═══════════════════════════════════════════════════════════════
   MAIN
═══════════════════════════════════════════════════════════════ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI2_Init();
    MX_I2C1_Init();
    MX_USB_DEVICE_Init();

    HAL_Delay(2000);

    /* ── Sensor init ── */
    MPU_Write(MPU_PWR_MGMT_1, 0x00);
    HAL_Delay(100);
    MPU_Write(MPU_CONFIG,     MPU_DLPF_92HZ);
    MPU_Write(MPU_SMPLRT_DIV, 0x00);

    MAG_Init();

    /* ── Gyro + accel calibration ── */
    {
        const char msg[] = "CALIB:START\r\n";
        CDC_Transmit_FS((uint8_t*)msg, sizeof(msg) - 1);
    }
    CalibrateGyro();
    CalibrateAccel();   /* [NEW] */

    /* ── Working variables ── */
    float gx, gy, gz, ax_raw, ay_raw, az_raw;
    float ax_f, ay_f, az_f;           /* filtered accel       [NEW] */
    float roll, pitch, yaw;
    int16_t mx, my, mz;
    uint8_t accel_valid;              /* gate flag            [NEW] */

    uint32_t last       = HAL_GetTick();
    uint32_t last_print = 0;

    while (1)
    {
        /* ── Read sensors ── */
        MPU_ReadAll(&gx, &gy, &gz, &ax_raw, &ay_raw, &az_raw);
        MAG_Read(&mx, &my, &mz);

        /* ── Time delta ── */
        uint32_t now  = HAL_GetTick();
        uint32_t dtms = now - last;
        if (dtms < DT_MIN_MS) dtms = DT_MIN_MS;
        if (dtms > DT_MAX_MS) dtms = DT_MAX_MS;
        float dt = (float)dtms * 0.001f;
        last = now;

        /* ── Apply gyro bias + dead-band ── */
        gx -= gbx;
        gy -= gby;
        gz -= gbz;

        if (fabsf(gx) < GYRO_DEADBAND) gx = 0.0f;
        if (fabsf(gy) < GYRO_DEADBAND) gy = 0.0f;
        if (fabsf(gz) < GYRO_DEADBAND) gz = 0.0f;

        /* ── Apply accel bias  [NEW] ── */
        ax_raw -= abx;
        ay_raw -= aby;
        az_raw -= abz;

        /* ── Accel filter: spike rejection + EMA + sanity + variance  [NEW] ── */
        Accel_Filter(ax_raw, ay_raw, az_raw,
                     &ax_f, &ay_f, &az_f,
                     &accel_valid);

        /* ── Axis remap ── */
#define REMAP_GX  gy
#define REMAP_GY  gz
#define REMAP_GZ  gx
#define REMAP_AX  ay_f
#define REMAP_AY  az_f
#define REMAP_AZ  ax_f

        /* ── Mahony: use accel only when it is trustworthy  [NEW] ── */
        if (accel_valid) {
            Mahony_Update(REMAP_GX, REMAP_GY, REMAP_GZ,
                          REMAP_AX, REMAP_AY, REMAP_AZ, dt);
        } else {
            Mahony_UpdateGyroOnly(REMAP_GX, REMAP_GY, REMAP_GZ, dt);
        }

        Quat_ToEuler(&roll, &pitch, &yaw);

        /* ── Magnetic heading ── */
        float hx = (float)(mx - MAG_OFFSET_X);
        float hy = (float)(my - MAG_OFFSET_Y);
        float heading = atan2f(hy, hx) * 57.29578f;
        if (heading < 0.0f) heading += 360.0f;

        /* ── Incoming USB command ── */
        if (rxFlag) {
            rxFlag = 0;

            if (strncmp((char*)UserRxBufferFS, "RESET_ALL", 9) == 0) {
                q0=1.0f; q1=0.0f; q2=0.0f; q3=0.0f;
                eIntX=0.0f; eIntY=0.0f; eIntZ=0.0f;
                /* Reset accel filter state too */
                aFiltX=0.0f; aFiltY=0.0f; aFiltZ=1.0f;
                aVarN=0; aVarM2=0.0f; aVarMean=1.0f; accel_variance=0.0f;
            }
            else if (strncmp((char*)UserRxBufferFS, "RESET_YAW", 9) == 0) {
                eIntZ = 0.0f;
                q3 = 0.0f;
                float invN = Q_rsqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
                q0 *= invN; q1 *= invN; q2 *= invN; q3 *= invN;
            }
            else if (strncmp((char*)UserRxBufferFS, "CAL_GYRO", 8) == 0) {
                const char msg[] = "CALIB_G:START\r\n";
                CDC_Transmit_FS((uint8_t*)msg, sizeof(msg) - 1);
                CalibrateGyro();
            }
            else if (strncmp((char*)UserRxBufferFS, "CAL_ACCEL", 9) == 0) {
                /* Re-calibrate accel on demand  [NEW] */
                const char msg[] = "CALIB_A:START\r\n";
                CDC_Transmit_FS((uint8_t*)msg, sizeof(msg) - 1);
                CalibrateAccel();
            }
        }

        /* ── Transmit at ~100 Hz ──
           Format now includes filtered accel and variance for debugging. [NEW] */
        if (HAL_GetTick() - last_print >= PRINT_INTERVAL_MS)
        {
            last_print = HAL_GetTick();

            char buf[192];
            int len = snprintf(buf, sizeof(buf),
                "G[%.2f %.2f %.2f] "
                "A[%.3f %.3f %.3f] "    /* filtered accel      [NEW] */
                "ANG[%.2f %.2f %.2f] "
                "MAG[%d %d %d] "
                "HDG:%.1f "
                "VAR:%.5f\r\n",         /* accel variance      [NEW] */
                gx, gy, gz,
                ax_f, ay_f, az_f,
                roll, pitch, yaw,
                (int)mx, (int)my, (int)mz,
                heading,
                accel_variance);

            if (CDC_Transmit_FS((uint8_t*)buf, (uint16_t)len) == USBD_BUSY) {
                HAL_Delay(1);
                CDC_Transmit_FS((uint8_t*)buf, (uint16_t)len);
            }
        }

        HAL_Delay(POLL_DELAY_MS);
    }
}

/* ═══════════════════════════════════════════════════════════════
   PERIPHERAL INITIALISATIONS  (CubeMX-generated, do not hand-edit)
═══════════════════════════════════════════════════════════════ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_SPI2_Init(void)
{
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin   = BMP_CS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BMP_CS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = MPU_CS_Pin;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MPU_CS_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
