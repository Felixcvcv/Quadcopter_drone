/*
 * sim/sensors.js — 虚拟 MPU6050 与 VL53L1X
 *
 * 关键点: 传感器输出的是**原始 int16 LSB 值**, 与固件里经过 I2C 读到的完全同级。
 * 这样 Com_Filter / Com_IMU 里的定点行为(低通返回 int16、加速度 1g≈16383 等)
 * 才能被真实地复现。
 *
 * 陀螺仪/加速度计都带常值零偏与白噪声 —— 零偏正是 AHRS 积分补偿项要消掉的东西,
 * 也是开机校准要标定的东西, 仿真里刻意保留。
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});
    var cfg = QC.config;
    var M = QC.math;
    var G = QC.dynamics.G;

    function makeSensors(seed, ground, getTruth) {
        var rng = M.Mulberry32(seed === undefined ? 20240921 : seed);
        var gauss = M.makeGaussian(rng);
        var S = cfg.SENSOR;

        var s = {
            /* 标定得到的零偏(对应 Inf_MPU6050.c 的 offsetGyroAccel) */
            offset: {
                gyro: { x: 0, y: 0, z: 0 },
                accel: { x: 0, y: 0, z: 0 }
            },
            /* 真实存在的零偏(传感器自身特性, 用来检验标定是否有效) */
            trueBias: {
                gyro: { x: S.gyroBiasDps.x, y: S.gyroBiasDps.y, z: S.gyroBiasDps.z },
                accel: { x: S.accelBiasG, y: S.accelBiasG * 0.6, z: -S.accelBiasG * 0.8 }
            },
            calibrated: false,

            /* 测距状态 */
            tof: {
                nowMs: 0,
                lastMm: 0,
                lastReadMs: -1e9,
                valid: false,
                dropouts: 0
            },

            /* 供 HUD 观察的原始值 */
            rawLast: { gx: 0, gy: 0, gz: 0, ax: 0, ay: 0, az: 0 }
        };

        /* ---------- 原始读数: 陀螺仪(LSB) ---------- */
        function rawGyro(out) {
            var t = getTruth();
            var R = M.qToMatrix(t.quat);
            /* 机体角速度 -> 芯片角速度: 这里 IMU 与机体同向(见 config.IMU_MOUNT) */
            var w = t.omega;
            var bx = w.x * cfg.RAD_TO_DEG, by = w.y * cfg.RAD_TO_DEG, bz = w.z * cfg.RAD_TO_DEG;

            out.x = (bx + s.trueBias.gyro.x + S.gyroNoiseDps * gauss() * 0.5) * cfg.MPU_GYRO_LSB_PER_DPS;
            out.y = (by + s.trueBias.gyro.y + S.gyroNoiseDps * gauss() * 0.5) * cfg.MPU_GYRO_LSB_PER_DPS;
            out.z = (bz + s.trueBias.gyro.z + S.gyroNoiseDps * gauss() * 0.5) * cfg.MPU_GYRO_LSB_PER_DPS;
            return out;
        }

        /* ---------- 原始读数: 加速度计(LSB), 测的是比力 ---------- */
        function rawAccel(out) {
            var t = getTruth();
            var R = M.qToMatrix(t.quat);
            /* 比力 = a_world - g_world, 再转到机体系 */
            var spec = { x: t.accel.x, y: t.accel.y, z: t.accel.z + G };
            var b = M.worldToBody(R, spec);

            out.x = (b.x / G) * cfg.MPU_ACCEL_LSB_PER_G + s.trueBias.accel.x * cfg.MPU_ACCEL_LSB_PER_G
                    + S.accelNoiseG * gauss() * cfg.MPU_ACCEL_LSB_PER_G;
            out.y = (b.y / G) * cfg.MPU_ACCEL_LSB_PER_G + s.trueBias.accel.y * cfg.MPU_ACCEL_LSB_PER_G
                    + S.accelNoiseG * gauss() * cfg.MPU_ACCEL_LSB_PER_G;
            out.z = (b.z / G) * cfg.MPU_ACCEL_LSB_PER_G + s.trueBias.accel.z * cfg.MPU_ACCEL_LSB_PER_G
                    + S.accelNoiseG * gauss() * cfg.MPU_ACCEL_LSB_PER_G;
            return out;
        }

        /* ---------- 对应 Inf_MPU6050_ReadGyroAccel(原始值) ---------- */
        function readGyroAccel(ga) {
            var g = { x: 0, y: 0, z: 0 }, a = { x: 0, y: 0, z: 0 };
            rawGyro(g);
            rawAccel(a);
            ga.gyro.gyroX = Math.round(g.x);
            ga.gyro.gyroY = Math.round(g.y);
            ga.gyro.gyroZ = Math.round(g.z);
            ga.accel.accelX = Math.round(a.x);
            ga.accel.accelY = Math.round(a.y);
            ga.accel.accelZ = Math.round(a.z);
            s.rawLast = {
                gx: ga.gyro.gyroX, gy: ga.gyro.gyroY, gz: ga.gyro.gyroZ,
                ax: ga.accel.accelX, ay: ga.accel.accelY, az: ga.accel.accelZ
            };
        }

        /* ---------- 对应 Inf_MPU6050_ReadGyroAccelCalibrated ---------- */
        function readGyroAccelCalibrated(ga) {
            readGyroAccel(ga);
            ga.gyro.gyroX -= s.offset.gyro.x;
            ga.gyro.gyroY -= s.offset.gyro.y;
            ga.gyro.gyroZ -= s.offset.gyro.z;
            ga.accel.accelX -= s.offset.accel.x;
            ga.accel.accelY -= s.offset.accel.y;
            ga.accel.accelZ -= s.offset.accel.z;
        }

        /**
         * 对应 Inf_MPU6050_Calibrate
         * 静止判定 + 255 次采样求平均, 得到六轴零偏.
         * 仿真里按"模拟时间"推进, 但对墙钟时间是瞬时的。
         * @param {function} advanceTime 可选: 推进模拟时间的回调(用于让静止判定有真实采样)
         */
        function calibrate(log) {
            var stillCount = 0, timeout = cfg.MPU_STILL_TIMEOUT;
            var current = { gyro: {}, accel: {} }, last = { gyro: {}, accel: {} };
            var i;

            readGyroAccel(last);
            while (stillCount < cfg.MPU_STILL_SAMPLE_COUNT && timeout > 0) {
                readGyroAccel(current);
                if (Math.abs(current.gyro.gyroX - last.gyro.gyroX) <= cfg.MPU_STILL_DELTA_THRESHOLD &&
                    Math.abs(current.gyro.gyroY - last.gyro.gyroY) <= cfg.MPU_STILL_DELTA_THRESHOLD &&
                    Math.abs(current.gyro.gyroZ - last.gyro.gyroZ) <= cfg.MPU_STILL_DELTA_THRESHOLD) {
                    stillCount++;
                } else {
                    stillCount = 0;
                }
                last.gyro.gyroX = current.gyro.gyroX;
                last.gyro.gyroY = current.gyro.gyroY;
                last.gyro.gyroZ = current.gyro.gyroZ;
                timeout--;
            }

            var sum = [0, 0, 0, 0, 0, 0];
            var sample = { gyro: {}, accel: {} };
            for (i = 0; i < cfg.MPU_CALIB_SAMPLE_COUNT; i++) {
                readGyroAccel(sample);
                sum[0] += sample.gyro.gyroX;
                sum[1] += sample.gyro.gyroY;
                sum[2] += sample.gyro.gyroZ;
                sum[3] += sample.accel.accelX;
                sum[4] += sample.accel.accelY;
                sum[5] += sample.accel.accelZ - cfg.MPU_ACCEL_1G_LSB;
            }

            var n = cfg.MPU_CALIB_SAMPLE_COUNT;
            s.offset.gyro.x = Math.round(sum[0] / n);
            s.offset.gyro.y = Math.round(sum[1] / n);
            s.offset.gyro.z = Math.round(sum[2] / n);
            s.offset.accel.x = Math.round(sum[3] / n);
            s.offset.accel.y = Math.round(sum[4] / n);
            s.offset.accel.z = Math.round(sum[5] / n);
            s.calibrated = true;

            if (log) {
                log('mpu6050: gyro bias (LSB)  x=' + s.offset.gyro.x +
                    ' y=' + s.offset.gyro.y + ' z=' + s.offset.gyro.z);
                log('mpu6050: accel offset (LSB) x=' + s.offset.accel.x +
                    ' y=' + s.offset.accel.y + ' z=' + s.offset.accel.z);
            }
            return s.offset;
        }

        /**
         * 推进传感器内部的时间基准(调度器每个时间片调用一次)
         * 固件里时间来自 SysTick, 这里由仿真时钟提供。
         */
        function advanceTime(dtMs) {
            s.tof.nowMs += dtMs;
        }

        /**
         * 对应 Inf_VL53LX1_GetHeight
         * 每 TOF_INTER_MEASUREMENT_MS 出一次新数据, 其余时间返回上次的值;
         * 超出量程或偶发丢包时视为"没有新数据", 同样返回上次的值。
         * @returns {number} 高度 mm
         */
        function tofRead() {
            var nowMs = s.tof.nowMs;

            if (nowMs - s.tof.lastReadMs >= cfg.TOF_INTER_MEASUREMENT_MS) {
                s.tof.lastReadMs = nowMs;

                var t = getTruth();
                /* 传感器在机体底部, 向下测到地面/楼顶/桥面 */
                var surface = ground.heightAt(t.position.x, t.position.y, t.position.z);
                var dist = (t.position.z - 0.02 - surface) * 1000; /* mm */

                if (rng() < cfg.TOF_DROPOUT_PROB || dist > cfg.TOF_MAX_RANGE_MM || dist < 0) {
                    /* 量程外或丢包: 保持上次结果 */
                    s.tof.dropouts++;
                } else {
                    s.tof.lastMm = Math.max(0, Math.round(dist + cfg.TOF_NOISE_MM * gauss()));
                    s.tof.valid = true;
                }
            }

            return s.tof.lastMm;
        }

        return {
            state: s,
            readGyroAccel: readGyroAccel,
            readGyroAccelCalibrated: readGyroAccelCalibrated,
            calibrate: calibrate,
            tofRead: tofRead,
            advanceTime: advanceTime
        };
    }

    QC.sensors = { makeSensors: makeSensors };
})(typeof globalThis !== 'undefined' ? globalThis : this);
