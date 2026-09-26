/*
 * ui/hud.js — 仪表盘
 *
 * 显示的都是仿真里的真实量(固件状态 + 物理状态), 不含演示用的假数据:
 *   姿态地平仪 / 姿态角 / 激光测距与定高目标 / Z轴速度估计
 *   遥控器 4 个通道值 / 4 路电机 PWM / 4 个 PID 输出
 *   4 个 LED (直接反映固件 ledTask 算出来的灯态)
 *   链路统计 + 姿态与高度波形
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});

    function $(id) { return document.getElementById(id); }

    function makeHud() {
        var el = {
            tagLink: $('tagLink'), tagArmed: $('tagArmed'), tagHold: $('tagHold'),
            tagPower: $('tagPower'), leds: $('leds'),
            statFrames: $('statFrames'), statLoss: $('statLoss'),
            statDist: $('statDist'), statFps: $('statFps'),
            vPitch: $('vPitch'), vRoll: $('vRoll'), vYaw: $('vYaw'),
            vHeight: $('vHeight'), vHold: $('vHold'), vZspeed: $('vZspeed'),
            vAgl: $('vAgl'), vVz: $('vVz'),
            vThr: $('vThr'), vYawS: $('vYawS'), vPitS: $('vPitS'), vRolS: $('vRolS'),
            vPidR: $('vPidR'), vPidP: $('vPidP'), vPidY: $('vPidY'), vPidZ: $('vPidZ'),
            motors: $('motors'),
            horizon: $('horizon'), scope: $('scope'),
            log: $('log'), logCount: $('logCount'),
            toast: $('toast'), hint: $('hint'),
            vbtnQ: $('vbtnQ'), vbtnE: $('vbtnE')
        };

        var motorSpans = [
            el.motors.querySelectorAll('.mbar')[0],
            el.motors.querySelectorAll('.mbar')[1],
            el.motors.querySelectorAll('.mbar')[2],
            el.motors.querySelectorAll('.mbar')[3]
        ].map(function (row) {
            return { bar: row.querySelector('.track span'), val: row.querySelector('b') };
        });

        var ledEls = {};
        Array.prototype.forEach.call(el.leds.querySelectorAll('i'), function (n) {
            ledEls[n.getAttribute('data-led')] = n;
        });

        var hz = el.horizon.getContext('2d');
        var sc = el.scope.getContext('2d');

        /* 波形缓冲区: 每 40ms 采一个点, 存 150 个 = 6 秒 */
        var TRACE_LEN = 150;
        var TRACE_DT = 40;
        var trace = {
            pitch: new Float32Array(TRACE_LEN),
            roll: new Float32Array(TRACE_LEN),
            alt: new Float32Array(TRACE_LEN),
            n: 0,
            lastMs: -1e9,
            maxAlt: 5
        };

        var logLines = 0;
        var toastTimer = null;
        var hintTimer = null;

        function fmt(v, d) { return (v === undefined || v === null || !isFinite(v)) ? '—' : v.toFixed(d); }

        function log(msg, cls) {
            var d = document.createElement('div');
            d.textContent = msg;
            if (cls) d.className = 'l-' + cls;
            el.log.appendChild(d);
            logLines++;
            el.logCount.textContent = '共 ' + logLines + ' 条';
            while (el.log.childElementCount > 220) el.log.removeChild(el.log.firstChild);
            el.log.scrollTop = el.log.scrollHeight;
        }

        function toast(msg, bad) {
            el.toast.textContent = msg;
            el.toast.className = 'show' + (bad ? ' bad' : '');
            if (toastTimer) clearTimeout(toastTimer);
            toastTimer = setTimeout(function () { el.toast.className = ''; }, 1900);
        }

        function hideHint() {
            if (el.hint.classList.contains('hide')) return;
            el.hint.classList.add('hide');
            if (hintTimer) clearTimeout(hintTimer);
            hintTimer = setTimeout(function () { el.hint.style.display = 'none'; }, 700);
        }

        /* ---------------- 姿态地平仪 ---------------- */
        function drawHorizon(pitch, roll, yaw) {
            var w = el.horizon.width, h = el.horizon.height;
            var cx = w / 2, cy = h / 2;
            var pxPerDeg = 1.9;

            hz.clearRect(0, 0, w, h);

            hz.save();
            hz.beginPath();
            hz.rect(0, 0, w, h);
            hz.clip();

            hz.translate(cx, cy);
            hz.rotate(-roll * Math.PI / 180);
            hz.translate(0, pitch * pxPerDeg);

            /* 天空 / 地面 */
            hz.fillStyle = '#3f7fb5';
            hz.fillRect(-w * 1.6, -h * 2.4, w * 3.2, h * 2.4);
            hz.fillStyle = '#6b5433';
            hz.fillRect(-w * 1.6, 0, w * 3.2, h * 2.4);

            /* 地平线 */
            hz.strokeStyle = 'rgba(255,255,255,0.85)';
            hz.lineWidth = 1.4;
            hz.beginPath();
            hz.moveTo(-w * 1.6, 0);
            hz.lineTo(w * 1.6, 0);
            hz.stroke();

            /* 俯仰刻度: 每 10 度一条 */
            hz.strokeStyle = 'rgba(255,255,255,0.5)';
            hz.lineWidth = 1;
            hz.font = '8px monospace';
            hz.fillStyle = 'rgba(255,255,255,0.6)';
            for (var d = -30; d <= 30; d += 10) {
                if (d === 0) continue;
                var y = -d * pxPerDeg;
                var half = (d % 20 === 0) ? 26 : 15;
                hz.beginPath();
                hz.moveTo(-half, y);
                hz.lineTo(half, y);
                hz.stroke();
                hz.fillText(String(Math.abs(d)), half + 3, y + 3);
            }
            hz.restore();

            /* 固定的机头符号 */
            hz.strokeStyle = '#ffcc33';
            hz.lineWidth = 2;
            hz.beginPath();
            hz.moveTo(cx - 36, cy);
            hz.lineTo(cx - 12, cy);
            hz.moveTo(cx + 12, cy);
            hz.lineTo(cx + 36, cy);
            hz.moveTo(cx, cy - 6);
            hz.lineTo(cx, cy + 4);
            hz.stroke();

            /* 航向角 */
            hz.fillStyle = 'rgba(255,255,255,0.82)';
            hz.font = '10px monospace';
            var yawTxt = (yaw % 360).toFixed(0) + '°';
            hz.fillText(yawTxt, cx - hz.measureText(yawTxt).width / 2, h - 5);
        }

        /* ---------------- 波形 ---------------- */
        function pushTrace(drone, dyn, simMs) {
            if (simMs - trace.lastMs < TRACE_DT) return;
            trace.lastMs = simMs;

            if (trace.n < TRACE_LEN) {
                trace.pitch[trace.n] = drone.eulerAngle.pitch;
                trace.roll[trace.n] = drone.eulerAngle.roll;
                trace.alt[trace.n] = dyn.position.z;
                trace.n++;
            } else {
                trace.pitch.copyWithin(0, 1); trace.pitch[TRACE_LEN - 1] = drone.eulerAngle.pitch;
                trace.roll.copyWithin(0, 1); trace.roll[TRACE_LEN - 1] = drone.eulerAngle.roll;
                trace.alt.copyWithin(0, 1); trace.alt[TRACE_LEN - 1] = dyn.position.z;
            }

            var mx = 1;
            for (var i = 0; i < trace.n; i++) mx = Math.max(mx, trace.alt[i]);
            trace.maxAlt = Math.max(5, Math.ceil(mx / 5) * 5);
        }

        function drawScope() {
            var w = el.scope.width, h = el.scope.height;
            sc.clearRect(0, 0, w, h);

            /* 网格 */
            sc.strokeStyle = 'rgba(120,150,180,0.16)';
            sc.lineWidth = 1;
            for (var gx = 0; gx <= 6; gx++) {
                var x = gx / 6 * w;
                sc.beginPath(); sc.moveTo(x, 0); sc.lineTo(x, h); sc.stroke();
            }
            for (var gy = 0; gy <= 4; gy++) {
                var y = gy / 4 * h;
                sc.beginPath(); sc.moveTo(0, y); sc.lineTo(w, y); sc.stroke();
            }
            /* 零度线 */
            sc.strokeStyle = 'rgba(120,150,180,0.4)';
            sc.beginPath(); sc.moveTo(0, h / 2); sc.lineTo(w, h / 2); sc.stroke();

            if (trace.n < 2) return;

            /* 姿态: ±45 度映射到整个高度 */
            var DEG = 45;
            function drawTrace(arr, color, mapY) {
                sc.strokeStyle = color;
                sc.lineWidth = 1.4;
                sc.beginPath();
                for (var i = 0; i < trace.n; i++) {
                    var x = i / (TRACE_LEN - 1) * w;
                    var y = mapY(arr[i]);
                    if (i === 0) sc.moveTo(x, y); else sc.lineTo(x, y);
                }
                sc.stroke();
            }

            drawTrace(trace.pitch, '#35b7e8', function (v) {
                return h / 2 - Math.max(-DEG, Math.min(DEG, v)) / DEG * (h / 2 - 4);
            });
            drawTrace(trace.roll, '#f2c14e', function (v) {
                return h / 2 - Math.max(-DEG, Math.min(DEG, v)) / DEG * (h / 2 - 4);
            });
            drawTrace(trace.alt, '#46d67f', function (v) {
                return h - 4 - Math.max(0, Math.min(trace.maxAlt, v)) / trace.maxAlt * (h - 8);
            });

            /* 刻度文字 */
            sc.fillStyle = 'rgba(200,215,230,0.5)';
            sc.font = '9px monospace';
            sc.fillText('+' + DEG + '°', 2, 9);
            sc.fillText('-' + DEG + '°', 2, h - 3);
            sc.fillText('0°', 2, h / 2 - 2);
            sc.fillText(trace.maxAlt.toFixed(0) + 'm', w - 24, 9);
        }

        /* ---------------- 更新 ---------------- */
        function update(rig, input, view, stats) {
            var drone = rig.drone, dyn = rig.dyn, cfg = rig.cfg;
            var COM_OK = QC.comm.Com_OK;

            /* 状态标签 */
            var connected = drone.isRemoteConnected === COM_OK;
            var armed = drone.isRemoteUnlocked === COM_OK;
            var hold = drone.isFixHeight === COM_OK;

            el.tagLink.textContent = connected ? '链路 已连接' : '链路 失联';
            el.tagLink.className = 'tag ' + (connected ? 'on' : 'bad');
            el.tagArmed.textContent = armed ? '已解锁' : '已上锁';
            el.tagArmed.className = 'tag ' + (armed ? 'warn' : '');
            el.tagHold.textContent = hold ? '定高 开' : '定高 关';
            el.tagHold.className = 'tag ' + (hold ? 'on' : '');
            var powered = drone.powerOn !== false;
            el.tagPower.textContent = powered ? '电源 开' : '电源 关';
            el.tagPower.className = 'tag ' + (powered ? 'on' : 'bad');

            /* LED: 直接读固件算出来的引脚电平 */
            ['leftTop', 'leftBottom', 'rightTop', 'rightBottom'].forEach(function (k) {
                var on = drone.leds[k].pinOn;
                if (on) ledEls[k].classList.add('on'); else ledEls[k].classList.remove('on');
            });

            /* 链路统计 */
            el.statFrames.textContent = '帧 ' + stats.framesOk + '/' + stats.framesSent;
            var lossPct = stats.framesSent ? (stats.framesLost / stats.framesSent * 100) : 0;
            el.statLoss.textContent = '丢包 ' + lossPct.toFixed(1) + '%' +
                (stats.framesCorrupt ? ' 错帧 ' + stats.framesCorrupt : '');
            el.statDist.textContent = '距离 ' + stats.distance.toFixed(0) + 'm';

            /* 姿态 */
            el.vPitch.textContent = fmt(drone.eulerAngle.pitch, 1) + '°';
            el.vRoll.textContent = fmt(drone.eulerAngle.roll, 1) + '°';
            el.vYaw.textContent = fmt(drone.eulerAngle.yaw, 1) + '°';
            drawHorizon(drone.eulerAngle.pitch, drone.eulerAngle.roll, drone.eulerAngle.yaw);

            /* 高度 */
            var tof = rig.sensors.state.tof.lastMm;
            el.vHeight.textContent = tof + ' mm';
            el.vHold.textContent = (drone.alt.status === 2) ? drone.alt.heightHold + ' mm' : '—';
            el.vZspeed.textContent = fmt(drone.debug.zSpeedEstimate, 0);
            var surface = QC.layout.heightAt(dyn.position.x, dyn.position.y, dyn.position.z);
            el.vAgl.textContent = fmt(dyn.position.z - surface, 2) + ' m';
            el.vVz.textContent = fmt(dyn.velocity.z, 2) + ' m/s';

            /* 遥控器通道: 显示飞控"收到"的值(即经过空口传输后的值) */
            el.vThr.textContent = drone.joyStick.THR;
            el.vYawS.textContent = drone.joyStick.YAW;
            el.vPitS.textContent = drone.joyStick.PIT;
            el.vRolS.textContent = drone.joyStick.ROL;

            /* 电机 */
            var mk = ['leftTop', 'leftBottom', 'rightTop', 'rightBottom'];
            for (var i = 0; i < 4; i++) {
                var sp = drone.motors[mk[i]].speed;
                motorSpans[i].bar.style.width = (sp / cfg.MOTOR_MAX * 100) + '%';
                motorSpans[i].val.textContent = sp;
            }

            /* PID 输出 */
            el.vPidR.textContent = fmt(drone.debug.rollOut, 1);
            el.vPidP.textContent = fmt(drone.debug.pitchOut, 1);
            el.vPidY.textContent = fmt(drone.debug.yawOut, 1);
            el.vPidZ.textContent = fmt(drone.debug.zPidOut, 1);

            /* 波形 */
            pushTrace(drone, dyn, rig.simMs);
            drawScope();

            /* 虚拟遥控器按钮高亮 */
            el.vbtnQ.className = 'vbtn' + (input.isHeld('btnLeftTop') ? ' active' : '');
            el.vbtnE.className = 'vbtn' + (input.isHeld('btnRightTop') ? ' active' : '');
        }

        return {
            update: update,
            log: log,
            toast: toast,
            hideHint: hideHint,
            resetTrace: function () { trace.n = 0; trace.lastMs = -1e9; trace.maxAlt = 5; },
            elements: el
        };
    }

    QC.hud = { makeHud: makeHud };
})(typeof globalThis !== 'undefined' ? globalThis : this);
