/*
 * main.js — 把各部分接起来: 渲染循环 + 固定步长仿真 + 交互
 *
 * 仿真采用**固定步长**: 每个渲染帧按真实耗时推进对应的模拟时间, 内部以 0.5ms
 * 为一个时间片。任务调度器按各自周期(4/6/50ms)触发任务, 所以控制周期始终是
 * 严格的 4ms —— PID 里的 dt 就来自这里, 一旦周期变了固件那组增益的行为也会变。
 */
(function (root) {
    'use strict';

    var QC = root.QC;
    var cfg = QC.config;

    /* ---------------- three.js 加载失败时给个明确提示 ---------------- */
    if (!root.THREE) {
        document.body.insertAdjacentHTML('beforeend',
            '<div style="position:fixed;inset:0;display:flex;align-items:center;justify-content:center;' +
            'background:#0d1015;color:#d8e2ec;font-size:15px;text-align:center;line-height:1.9;z-index:99">' +
            '<div>加载 three.js 失败。<br>本仿真器通过 CDN 引用 three.js, 首次打开需要联网。<br>' +
            '<span style="color:#8fa0b2;font-size:13px">' +
            '离线用法: 下载 three.min.js 放到 js/vendor/ 下, 并把 index.html 里的 script 地址改成本地路径。' +
            '</span></div></div>');
        return;
    }

    var hud = QC.hud.makeHud();
    var input = QC.input.makeInput();

    /* ---------------- 渲染器与场景 ---------------- */
    var viewport = document.getElementById('viewport');
    var renderer = new THREE.WebGLRenderer({
        canvas: viewport, antialias: true, powerPreference: 'high-performance'
    });
    renderer.setPixelRatio(Math.min(root.devicePixelRatio || 1, 2));
    renderer.setSize(root.innerWidth, root.innerHeight, false);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputEncoding = THREE.sRGBEncoding;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 0.92;

    var camera = new THREE.PerspectiveCamera(62, root.innerWidth / root.innerHeight, 0.08, 2200);
    camera.up.set(0, 0, 1);   /* Z 轴向上, 与固件文档里的地理坐标系一致 */

    var world = QC.world.build();

    /* ---------------- 飞控 + 遥控器 + 物理 ---------------- */
    var drone = QC.state.makeDroneState();
    var remote = QC.state.makeRemoteState();
    var dyn = QC.dynamics.makeDynamics();
    var remotePos = { x: 6, y: -6, z: 1.6 };

    var sensors = QC.sensors.makeSensors(20240921, QC.layout, function () {
        return {
            position: dyn.position, velocity: dyn.velocity,
            quat: dyn.quat, omega: dyn.omega, accel: dyn.accelWorld
        };
    });

    var scheduler = QC.tasks.makeScheduler(0.5);
    scheduler.link = QC.comm.LinkModel();
    var linkRng = QC.math.Mulberry32(31337);

    /* ---------------- 运行状态 ---------------- */
    var S = {
        simMs: 0,
        camMode: 0,          /* 0 追尾  1 环绕  2 机载 */
        camDist: 6.5,
        orbitYaw: -0.75,
        orbitPitch: 0.36,
        dragging: false,
        pointer: { x: 0, y: 0 },
        timeScale: 1,
        timeScaleList: [1, 0.5, 0.25],
        timeScaleIdx: 0,
        fps: 60,
        lastFrameMs: 0,
        booted: false,
        armScript: null,
        crashNotified: false,
        stats: { framesOk: 0, framesSent: 0, framesLost: 0, framesCorrupt: 0, distance: 0 }
    };

    /* ---------------- 上电流程 (对应固件的启动顺序) ---------------- */
    function boot() {
        hud.log('=== 四轴无人机仿真器 ===');
        hud.log('飞控算法移植自 firmware/flight_controller, 控制周期 ' +
            cfg.FLIGHT_EXEC_CYCLE + 'ms, 通讯周期 ' + cfg.COMMUNICATION_EXEC_CYCLE + 'ms');
        hud.log('comm: 2.4G 模块自检 (通道 ' + cfg.RF_CHANNEL + ', 2Mbps) ... 通过');
        hud.log('comm: 配置为接收模式');
        hud.log('motor: 4 路 PWM 启动');
        hud.log('vl53l1x: 初始化, 单次测量 ' + cfg.TOF_TIMING_BUDGET_MS + 'ms, 量程 ' +
            (cfg.TOF_MAX_RANGE_MM / 1000).toFixed(0) + 'm');

        /* 六轴零偏校准: 固件在调度器启动前阻塞完成, 要求机身水平静止 */
        sensors.calibrate(function (s) { hud.log(s); });
        hud.log('mpu6050: 校准完成; 残余漂移由 AHRS 积分项继续抑制');
        hud.log('姿态解算: 四元数 + 加速度计 PI 补偿, Kp=' + cfg.AHRS_KP + ' Ki=' + cfg.AHRS_KI);
        hud.log('--- 上电完成, 等待遥控器数据 ---');
        hud.log('按 空格 自动执行解锁序列: 油门最高 1.2s -> 最低 1.2s');
    }

    /* ---------------- 复位 ---------------- */
    function resetAll(msg) {
        QC.dynamics.reset(dyn, QC.layout.PAD.x, QC.layout.PAD.y, QC.layout.PAD.z);
        QC.imu.Reset();

        drone.isRemoteConnected = QC.comm.Com_FAIL;
        drone.isRemoteUnlocked = QC.comm.Com_FAIL;
        drone.isFixHeight = QC.comm.Com_FAIL;
        drone.joyStick.isFixHeight = 0;
        drone.joyStick.isPowerDown = 0;
        drone.alt.status = 0;
        drone.alt.cnt = 0;
        drone.alt.staticAcc = 0;
        drone.alt.heightHold = 0;
        drone.alt.thrHold = 0;
        drone.lastHeight = 0;
        drone.lastGyro[0] = drone.lastGyro[1] = drone.lastGyro[2] = 0;
        drone.powerOn = true;
        drone.powerNotify = false;
        drone.crashed = false;
        drone.leds.firstUnlock = 1;
        drone.ledCnt = 0;
        drone.attitudeDiverged = 0;
        drone.comm.lostCnt = cfg.LOST_FRAME_LIMIT;
        drone.comm.remoteStatus = QC.comm.THR_FREE;
        drone.comm.thrMaxDuration = 0;
        drone.comm.thrMinDuration = 0;
        drone.comm.lowDuration = 0;

        Object.keys(drone.pids).forEach(function (k) { QC.pid.Reset(drone.pids[k]); });
        drone.kfs.forEach(function (kf) {
            kf.lastP = 0; kf.nowP = 0; kf.estimate = 0; kf.gain = 0;
        });

        sensors.state.tof.lastMm = 0;
        sensors.state.tof.lastReadMs = -1e9;
        sensors.state.tof.dropouts = 0;
        sensors.calibrate();          /* 此时飞机静止在起降场, 重新标一次零偏 */

        scheduler.accum.flight = scheduler.accum.comm = 0;
        scheduler.accum.led = scheduler.accum.power = 0;
        scheduler.stats.framesOk = scheduler.stats.framesSent = 0;
        scheduler.stats.framesLost = scheduler.stats.framesCorrupt = 0;

        input.axes.THR = 0;
        input.axes.YAW = input.axes.PIT = input.axes.ROL = 500;

        S.armScript = null;
        S.crashNotified = false;
        S.simMs = 0;
        hud.resetTrace();
        hud.toast(msg || '已复位');
        hud.log('--- 复位: 电机停转, 重新校准零偏 ---');
    }

    /* ---------------- 仿真推进 (固定 0.5ms 时间片) ---------------- */
    function stepSim(dtMs) {
        var remain = dtMs;
        while (remain > 1e-9) {
            var dt = Math.min(0.5, remain);
            remain -= dt;
            S.simMs += dt;

            /* 自动解锁序列: 临时接管油门通道 */
            if (S.armScript) {
                S.armScript.t += dt;
                if (S.armScript.t < 1300) input.axes.THR = 1000;
                else if (S.armScript.t < 2600) input.axes.THR = 0;
                else {
                    S.armScript = null;
                    hud.toast('解锁序列执行完毕');
                }
            }

            input.applyToRemote(remote, dt);

            QC.tasks.advance(scheduler, dt, drone, remote, sensors,
                dyn.position, remotePos, linkRng);

            QC.dynamics.readMotorCommands(dyn, drone.motors);
            QC.dynamics.step(dyn, dt / 1000, QC.layout);

            /* 撞上建筑/桥塔 */
            if (!dyn.crashed) {
                var hit = QC.layout.hitTest(dyn.position.x, dyn.position.y, dyn.position.z);
                if (hit) {
                    dyn.crashed = true;
                    dyn.crashReason = '撞上' + hit.name;
                    drone.crashed = true;
                }
            }
            if (dyn.crashed && !S.crashNotified) {
                S.crashNotified = true;
                hud.toast(dyn.crashReason, true);
                hud.log('!! ' + dyn.crashReason + ' —— 按 R 复位', 'bad');
            }

            /* 出界保护: 别让飞机飞出场景 */
            var lim = cfg.WORLD.size / 2 - 20;
            if (Math.abs(dyn.position.x) > lim || Math.abs(dyn.position.y) > lim) {
                dyn.position.x = QC.math.clamp(dyn.position.x, -lim, lim);
                dyn.position.y = QC.math.clamp(dyn.position.y, -lim, lim);
                dyn.velocity.x = 0;
                dyn.velocity.y = 0;
            }
        }
    }

    /* ---------------- 相机 ---------------- */
    var camPos = new THREE.Vector3(0, 0, 0);
    var camLook = new THREE.Vector3();
    var camWant = new THREE.Vector3();

    function updateCamera(dtS) {
        var R = QC.math.qToMatrix(dyn.quat);
        var fwd = { x: R[0][0], y: R[1][0], z: R[2][0] };   /* 机头方向(世界系) */

        if (S.camMode === 0) {
            /* 追尾: 在机头反方向、略高处, 平滑跟随 */
            /* 相机在机头反方向略高处, 视线指向机头前方一点,
               这样地平线大约落在画面上三分之一处 */
            var back = S.camDist * 0.95;
            camWant.set(dyn.position.x - fwd.x * back, dyn.position.y - fwd.y * back,
                dyn.position.z + S.camDist * 0.24 + 0.55);
            if (camPos.lengthSq() === 0) camPos.copy(camWant);
            camPos.lerp(camWant, Math.min(1, dtS * 4.0));
            camera.position.copy(camPos);
            camLook.set(dyn.position.x + fwd.x * (back * 0.75 + 2.5),
                dyn.position.y + fwd.y * (back * 0.75 + 2.5),
                dyn.position.z + 0.35);
            camera.lookAt(camLook);
        } else if (S.camMode === 1) {
            /* 环绕: 拖动鼠标转, 滚轮缩放 */
            var cp = Math.cos(S.orbitPitch), sp = Math.sin(S.orbitPitch);
            camWant.set(
                dyn.position.x + Math.cos(S.orbitYaw) * cp * S.camDist,
                dyn.position.y + Math.sin(S.orbitYaw) * cp * S.camDist,
                dyn.position.z + sp * S.camDist);
            if (camPos.lengthSq() === 0) camPos.copy(camWant);
            camPos.lerp(camWant, Math.min(1, dtS * 7));
            camera.position.copy(camPos);
            camera.lookAt(dyn.position.x, dyn.position.y, dyn.position.z + 0.1);
        } else {
            /* 机载: 坐在机头往前看 */
            camera.position.set(
                dyn.position.x + fwd.x * 0.1,
                dyn.position.y + fwd.y * 0.1,
                dyn.position.z + fwd.z * 0.1);
            camLook.set(dyn.position.x + fwd.x * 6,
                dyn.position.y + fwd.y * 6,
                dyn.position.z + fwd.z * 6);
            camera.lookAt(camLook);
        }
        camera.up.set(0, 0, 1);
    }

    /* ---------------- 状态同步到模型 ---------------- */
    var MOTOR_KEYS = ['leftTop', 'leftBottom', 'rightTop', 'rightBottom'];
    var tmpQ = new THREE.Quaternion();

    function syncModels(dtS) {
        world.drone.group.position.set(dyn.position.x, dyn.position.y, dyn.position.z);
        /* 仿真用 (w,x,y,z), Three.js 用 (x,y,z,w) */
        tmpQ.set(dyn.quat.q1, dyn.quat.q2, dyn.quat.q3, dyn.quat.q0);
        world.drone.group.quaternion.copy(tmpQ);

        for (var i = 0; i < world.drone.props.length; i++) {
            var pr = world.drone.props[i];
            var sp = drone.motors[MOTOR_KEYS[i]].speed / cfg.MOTOR_MAX;
            pr.mesh.rotation.z += (0.4 + sp * 3.6) * 26 * dtS * pr.dir;
        }

        MOTOR_KEYS.forEach(function (k) {
            /* 固件里 LED 低电平点亮, 所以 pinOn=false 时是亮的 */
            world.drone.ledMats[k].color.setHex(drone.leds[k].pinOn ? 0x241010 : 0xff4a3d);
        });

        /* 太阳跟着飞机, 保证阴影贴图始终覆盖飞机周围 */
        world.sun.position.set(dyn.position.x + 180, dyn.position.y - 150, dyn.position.z + 260);
        world.sun.target.position.set(dyn.position.x, dyn.position.y, dyn.position.z);
        world.sun.target.updateMatrixWorld();

        var dx = dyn.position.x - remotePos.x;
        var dy = dyn.position.y - remotePos.y;
        var dz = dyn.position.z - remotePos.z;
        S.stats.distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
        S.stats.framesOk = scheduler.stats.framesOk;
        S.stats.framesSent = scheduler.stats.framesSent;
        S.stats.framesLost = scheduler.stats.framesLost;
        S.stats.framesCorrupt = scheduler.stats.framesCorrupt;
    }

    /* ---------------- 一次性命令 ---------------- */
    function handleCommands() {
        var c = input.cmd;

        if (c.reset) { c.reset = 0; resetAll('已复位'); }
        if (c.toggleJam) {
            c.toggleJam = 0;
            scheduler.link.jammed = !scheduler.link.jammed;
            hud.toast(scheduler.link.jammed ? '已注入 2.4G 干扰 (模拟断链)' : '干扰已解除',
                scheduler.link.jammed);
            hud.log('link: ' + (scheduler.link.jammed ? '注入 2.4G 干扰' : '干扰解除'));
        }
        if (c.toggleHud) { c.toggleHud = 0; document.body.classList.toggle('hud-hidden'); }
        if (c.cam >= 0) {
            S.camMode = c.cam;
            c.cam = -1;
            hud.toast(['追尾视角', '环绕视角 (拖动旋转 / 滚轮缩放)', '机载视角'][S.camMode]);
        }
        if (c.timeScale) {
            c.timeScale = 0;
            S.timeScaleIdx = (S.timeScaleIdx + 1) % S.timeScaleList.length;
            S.timeScale = S.timeScaleList[S.timeScaleIdx];
            hud.toast('时间倍速 ' + S.timeScale + 'x (观察控制环路时很有用)');
        }
        if (c.armAuto) {
            c.armAuto = 0;
            if (!dyn.crashed && drone.powerOn !== false) {
                S.armScript = { t: 0 };
                hud.toast('执行解锁序列: 油门最高 1.2s -> 最低 1.2s');
                hud.log('input: 自动解锁序列开始');
            }
        }
        if (c.calibrate) {
            c.calibrate = 0;
            /* 长按右上键: 重算摇杆零偏 (对应 App_DataProcess_JoyStickCalcBias) */
            var n = cfg.CALIB_SAMPLE_COUNT;
            var sumP = 0, sumR = 0;
            for (var i = 0; i < n; i++) {
                sumP += input.axes.PIT - cfg.STICK_VALUE_MID;
                sumR += input.axes.ROL - cfg.STICK_VALUE_MID;
            }
            remote.joyStickBias.PIT = Math.round(sumP / n);
            remote.joyStickBias.ROL = Math.round(sumR / n);
            hud.toast('摇杆零偏已重新校准');
            hud.log('joystick: 零偏校准 pit=' + remote.joyStickBias.PIT +
                ' rol=' + remote.joyStickBias.ROL);
        }

        while (input.events.length) hud.log('key: ' + input.events.shift());
    }

    /* ---------------- 主循环 ---------------- */
    function frame(nowMs) {
        if (!S.lastFrameMs) S.lastFrameMs = nowMs;
        var dtReal = Math.min(nowMs - S.lastFrameMs, 100);
        S.lastFrameMs = nowMs;
        if (dtReal > 0) S.fps = S.fps * 0.92 + (1000 / dtReal) * 0.08;

        if (!S.booted) { S.booted = true; boot(); }

        handleCommands();

        /* 摇杆积分按整帧时间算(与帧率无关), 再按固定步长推进仿真 */
        if (!S.armScript) input.update(dtReal * S.timeScale);
        var budget = Math.min(dtReal * S.timeScale, 60);
        stepSim(budget);

        var dtS = Math.min(dtReal, 100) / 1000;
        syncModels(dtS);
        updateCamera(dtS);
        renderer.render(world.scene, camera);

        hud.update({
            drone: drone, dyn: dyn, cfg: cfg, sensors: sensors,
            simMs: S.simMs
        }, input, null, S.stats);
        hud.elements.statFps.textContent = S.fps.toFixed(0) + ' fps' +
            (S.timeScale !== 1 ? ' · ' + S.timeScale + 'x' : '');

        requestAnimationFrame(frame);
    }

    /* ---------------- 事件 ---------------- */
    root.addEventListener('keydown', function (e) {
        input.onKeyDown(e);
        hud.hideHint();
    });
    root.addEventListener('keyup', function (e) { input.onKeyUp(e); });

    viewport.addEventListener('pointerdown', function (e) {
        S.dragging = true;
        S.pointer.x = e.clientX;
        S.pointer.y = e.clientY;
        try { viewport.setPointerCapture(e.pointerId); } catch (err) { /* 忽略 */ }
        hud.hideHint();
    });
    viewport.addEventListener('pointermove', function (e) {
        if (!S.dragging) return;
        var dx = e.clientX - S.pointer.x;
        var dy = e.clientY - S.pointer.y;
        S.pointer.x = e.clientX;
        S.pointer.y = e.clientY;
        if (S.camMode !== 1) { S.camMode = 1; hud.toast('环绕视角'); }
        S.orbitYaw -= dx * 0.006;
        S.orbitPitch = QC.math.clamp(S.orbitPitch + dy * 0.005, -0.15, 1.35);
    });
    viewport.addEventListener('pointerup', function (e) {
        S.dragging = false;
        try { viewport.releasePointerCapture(e.pointerId); } catch (err) { /* 忽略 */ }
    });
    viewport.addEventListener('wheel', function (e) {
        e.preventDefault();
        var lo = (S.camMode === 1) ? 1.5 : 2.5;
        var hi = (S.camMode === 1) ? 90 : 26;
        S.camDist = QC.math.clamp(S.camDist * (1 + Math.sign(e.deltaY) * 0.12), lo, hi);
    }, { passive: false });
    root.addEventListener('resize', function () {
        renderer.setSize(root.innerWidth, root.innerHeight, false);
        camera.aspect = root.innerWidth / root.innerHeight;
        camera.updateProjectionMatrix();
    });

    QC.input.bindVirtualSticks(input, document);

    /* ---------------- 起步 ---------------- */
    QC.dynamics.reset(dyn, QC.layout.PAD.x, QC.layout.PAD.y, QC.layout.PAD.z);
    requestAnimationFrame(frame);

    /* 供浏览器自动化测试/控制台调试使用 */
    root.QCSim = {
        QC: QC, drone: drone, remote: remote, dyn: dyn, sensors: sensors,
        scheduler: scheduler, input: input, state: S, hud: hud,
        renderer: renderer, scene: world.scene, camera: camera, world: world,
        resetAll: resetAll,
        hoverThrottle: function () { return QC.dynamics.hoverSpeed() / cfg.THROTTLE_GAIN; },
        /* 锁定通道(自动化演示用): 传 null 解除锁定 */
        setSticks: function (o) {
            ['THR', 'YAW', 'PIT', 'ROL'].forEach(function (k) {
                if (o[k] !== undefined) input.pinAxis(k, o[k]);
            });
        },
        releaseSticks: function () { input.unpinAll(); },
        /* 直接发送键盘事件, 用来验证键盘通路 */
        pressKey: function (code, type) {
            root.dispatchEvent(new KeyboardEvent(type || 'keydown', { code: code, bubbles: true }));
        },
        advance: function (ms) { stepSim(ms); },
        boot: function () { if (!S.booted) { S.booted = true; boot(); } }
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
