/*
 * sim/dynamics.js — 六自由度刚体动力学
 *
 * 仿真器自己引入的部分: 固件里只有"给电机写 PWM", 没有飞机的物理模型。
 * 模型包含:
 *   - 电机一阶惯性 (空心杯电机 + 小桨的机械时间常数)
 *   - 螺旋桨推力与反扭矩 (推力 ∝ 转速²)
 *   - 欧拉方程转动 (含 ω×(Iω) 陀螺耦合项与转动阻尼)
 *   - 平动: 升力 + 重力 + 气动阻力
 *   - 地面/楼顶/桥面碰撞 与 坠机判定
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});
    var cfg = QC.config;
    var M = QC.math;

    var G = 9.80665; /* m/s^2 */

    /*
     * 电机在机体坐标系中的位置(单位: 轴距系数 ±1) 与"反扭矩极性" c
     *
     * 机体系: X 前, Y 右, Z 上.  四个电机的坐标都是 (±a, ±a), a = 轴距/√2.
     *
     * 这组取值不是随便定的, 而是由固件 App_Flight_MotorWithPosturePID 的混控
     * 反推出来的:
     *   横滚 +: leftTop/leftBottom   -> 这两个必须在滚转轴的两侧
     *   俯仰 +: leftTop/rightTop     -> 这两个必须在俯仰轴的两侧
     *   偏航 +: leftTop/rightBottom  -> 对角同组, 与另一对角反向
     * 把这几条和"控制器极性必须构成负反馈"联立, 就能唯一确定:
     *   - 固件里的 "left" 那一对落在本仿真的 +Y 侧, "top" 那一对落在 +X 侧
     *     (固件的 L/R 只是命名, 真正决定几何的是混控的分组方式)
     *   - leftTop/rightBottom 与 leftBottom/rightTop 的旋向相反
     * 于是 +roll/+pitch/+yaw 分别产生绕 +X/+Y/+Z 的正向力矩, 与姿态环自洽。
     */
    var MOTOR_LAYOUT = {
        leftTop:     { x: +1, y: +1, c: +1 }, /* 固件的"左上" */
        leftBottom:  { x: -1, y: +1, c: -1 }, /* 固件的"左下" */
        rightTop:    { x: +1, y: -1, c: -1 }, /* 固件的"右上" */
        rightBottom: { x: -1, y: -1, c: +1 }  /* 固件的"右下" */
    };
    var MOTOR_KEYS = ['leftTop', 'leftBottom', 'rightTop', 'rightBottom'];

    function makeDynamics() {
        var D = cfg.DRONE;
        var a = D.armLength / Math.SQRT2; /* 电机坐标 = ±a */

        return {
            mass: D.mass,
            /* 转动惯量对角阵 */
            inertia: { x: D.inertia.x, y: D.inertia.y, z: D.inertia.z },

            position: { x: 0, y: 0, z: 0 },          /* 世界系, m */
            velocity: { x: 0, y: 0, z: 0 },          /* 世界系, m/s */
            accelWorld: { x: 0, y: 0, z: 0 },        /* 世界系加速度(含重力) */
            quat: M.qIdentity(),                     /* 机体系 -> 世界系 */
            omega: { x: 0, y: 0, z: 0 },             /* 机体系角速度, rad/s */

            motorActual: { leftTop: 0, leftBottom: 0, rightTop: 0, rightBottom: 0 },
            motorCmd: { leftTop: 0, leftBottom: 0, rightTop: 0, rightBottom: 0 },

            /* 状态标志 */
            onGround: true,
            crashed: false,
            crashReason: '',
            lastImpactSpeed: 0,

            /* 用于 HUD 显示 */
            thrust: 0,
            torque: { x: 0, y: 0, z: 0 },

            motorLayout: MOTOR_LAYOUT,
            motorKeys: MOTOR_KEYS
        };
    }

    function reset(d, x, y, z) {
        d.position.x = x || 0;
        d.position.y = y || 0;
        d.position.z = z || 0;
        d.velocity.x = d.velocity.y = d.velocity.z = 0;
        d.quat = M.qIdentity();
        d.omega.x = d.omega.y = d.omega.z = 0;
        for (var i = 0; i < MOTOR_KEYS.length; i++) {
            d.motorActual[MOTOR_KEYS[i]] = 0;
            d.motorCmd[MOTOR_KEYS[i]] = 0;
        }
        d.onGround = true;
        d.crashed = false;
        d.crashReason = '';
        d.lastImpactSpeed = 0;
        d.thrust = 0;
        d.torque.x = d.torque.y = d.torque.z = 0;
    }

    /* 从飞控的电机结构体读取指令 (对应固件"写定时器比较寄存器") */
    function readMotorCommands(d, motors) {
        d.motorCmd.leftTop = motors.leftTop.speed;
        d.motorCmd.leftBottom = motors.leftBottom.speed;
        d.motorCmd.rightTop = motors.rightTop.speed;
        d.motorCmd.rightBottom = motors.rightBottom.speed;
    }

    /**
     * 推进一步
     * @param {number} dt 秒
     * @param {object} ground 提供 heightAt(x, y) 与 collideVertical 的世界接口
     */
    function step(d, dt, ground) {
        var D = cfg.DRONE;
        var a = D.armLength / Math.SQRT2;

        /* 记下积分前的速度: 末尾要用速度的实际变化量反算加速度。
           这样地面接触力也会被计入, 加速度计才不会在停机坪上读到 0g。
           (加速度计测的是比力 a - g: 静止在地面时飞机加速度为 0,
            所以读数应该是 +1g, 而不是自由落体的 0。) */
        var vx0 = d.velocity.x, vy0 = d.velocity.y, vz0 = d.velocity.z;

        /* ---------- 1. 电机一阶惯性: w += (cmd - w) * (1 - e^(-dt/tau)) ---------- */
        var alpha = 1 - Math.exp(-dt / D.motorTimeConstant);
        for (var i = 0; i < MOTOR_KEYS.length; i++) {
            var k = MOTOR_KEYS[i];
            d.motorActual[k] += (d.motorCmd[k] - d.motorActual[k]) * alpha;
        }

        /* ---------- 2. 推力与力矩 ---------- */
        var Fz = 0, tauX = 0, tauY = 0, tauZ = 0;
        for (i = 0; i < MOTOR_KEYS.length; i++) {
            k = MOTOR_KEYS[i];
            var L = MOTOR_LAYOUT[k];
            var w = d.motorActual[k];
            var T = D.thrustCoef * w * w;      /* 推力 N */
            Fz += T;
            tauX += T * (L.y * a);             /* τ = r × F, r=(x,y,0), F=(0,0,T) */
            tauY += -T * (L.x * a);
            tauZ += L.c * D.torqueCoef * w * w; /* 螺旋桨反扭矩 */
        }
        d.thrust = Fz;
        d.torque.x = tauX;
        d.torque.y = tauY;
        d.torque.z = tauZ;

        /* ---------- 3. 转动: 欧拉方程 I·ω̇ = τ - ω×(Iω) - 阻尼 ---------- */
        var Ix = d.inertia.x, Iy = d.inertia.y, Iz = d.inertia.z;
        var wx = d.omega.x, wy = d.omega.y, wz = d.omega.z;
        var Iwx = Ix * wx, Iwy = Iy * wy, Iwz = Iz * wz;
        var cx = wy * Iwz - wz * Iwy;
        var cy = wz * Iwx - wx * Iwz;
        var cz = wx * Iwy - wy * Iwx;
        var cd = D.angularDrag;

        d.omega.x += dt * (tauX - cx - cd * wx) / Ix;
        d.omega.y += dt * (tauY - cy - cd * wy) / Iy;
        d.omega.z += dt * (tauZ - cz - cd * wz) / Iz;

        /* ---------- 4. 四元数积分 ---------- */
        d.quat = M.qIntegrate(d.quat, d.omega, dt);
        var R = M.qToMatrix(d.quat);

        /* ---------- 5. 平动 ---------- */
        var Fworld = M.bodyToWorld(R, { x: 0, y: 0, z: Fz });

        /* 气动阻力: 水平与垂直分开给系数(垂直方向还有桨盘阻尼) */
        var vx = d.velocity.x, vy = d.velocity.y, vz = d.velocity.z;
        var dragX = -D.linearDrag * vx;
        var dragY = -D.linearDrag * vy;
        var dragZ = -D.verticalDrag * vz;

        var ax = Fworld.x / d.mass + dragX / d.mass;
        var ay = Fworld.y / d.mass + dragY / d.mass;
        var az = Fworld.z / d.mass + dragZ / d.mass - G;

        d.velocity.x += ax * dt;
        d.velocity.y += ay * dt;
        d.velocity.z += az * dt;

        d.position.x += d.velocity.x * dt;
        d.position.y += d.velocity.y * dt;
        d.position.z += d.velocity.z * dt;

        /* ---------- 6. 地面 / 楼顶 / 桥面 碰撞 ---------- */
        if (ground) {
            var gh = ground.heightAt(d.position.x, d.position.y, d.position.z);
            var skin = 0.02; /* 起落架高度 */
            if (d.position.z <= gh + skin) {
                var impact = -d.velocity.z;
                if (impact > 0) d.lastImpactSpeed = impact;

                d.position.z = gh + skin;
                if (d.velocity.z < 0) d.velocity.z = 0;

                /* 落地后水平速度快速衰减(摩擦), 角速度也衰减 */
                var friction = Math.exp(-dt / 0.15);
                d.velocity.x *= friction;
                d.velocity.y *= friction;
                d.omega.x *= friction;
                d.omega.y *= friction;
                d.omega.z *= friction;

                d.onGround = true;

                /* 硬着陆判定: 垂直速度过大或落地时姿态倾斜过大 */
                if (impact > 3.2 || Math.abs(d.omega.x) > 6 || Math.abs(d.omega.y) > 6) {
                    d.crashed = true;
                    d.crashReason = '着陆过重 (垂直速度 ' + impact.toFixed(1) + ' m/s)';
                }
            } else {
                d.onGround = false;
            }
        }

        /* ---------- 7. 用速度的实际变化量反算世界系加速度 ----------
           加速度计测的是比力 a - g。静止在地面(含楼顶/桥面)时飞机加速度为 0,
           所以读数应当是 +1g; 若直接用自由落体加速度(-g)去算, 会比力变成 0g,
           姿态解算会因此发散、定高的重力基准也会标错。 */
        d.accelWorld.x = (d.velocity.x - vx0) / dt;
        d.accelWorld.y = (d.velocity.y - vy0) / dt;
        d.accelWorld.z = (d.velocity.z - vz0) / dt;
    }

    /* 悬停转速: 用于文档与自检 (推力总和 = 重力) */
    function hoverSpeed() {
        var D = cfg.DRONE;
        return Math.sqrt(D.mass * G / (4 * D.thrustCoef));
    }

    QC.dynamics = {
        G: G,
        MOTOR_LAYOUT: MOTOR_LAYOUT,
        MOTOR_KEYS: MOTOR_KEYS,
        makeDynamics: makeDynamics,
        reset: reset,
        readMotorCommands: readMotorCommands,
        step: step,
        hoverSpeed: hoverSpeed
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
