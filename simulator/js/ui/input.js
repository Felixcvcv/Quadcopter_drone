/*
 * ui/input.js — 键盘当手柄, 外加一个能用鼠标拖的虚拟遥控器
 *
 * 模拟的是**遥控器本身**, 不是飞机:
 *   - 摇杆: 4 个通道产生 0~1000 的行程量(与固件里 joyStick 的含义一致)。
 *     油门松手保持, 其余三个通道松手自动回中 —— 和真摇杆的弹簧一样。
 *   - 按键: 按下记时刻, 松开时按按住时长判定短按/长按, 与固件
 *     Inf_JoyStickAndKey_KeyScan 的判定一致(>500ms 算长按)。
 *   - 极性: 摇杆方向到通道值的映射由 config.STICK_POLARITY 决定,
 *     对应真实遥控器上电位器的接线方向。
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});

    /* 摇杆推到底需要的时间: 900/秒 -> 约 1.1 秒走完 1000 */
    var AXIS_RATE = 900;
    var AXIS_RATE_FINE = 260;   /* 按住 Shift 时的微调速度 */
    var LONG_PRESS_MS = 500;    /* 与 remote_controller 的 KEY_LONG_PRESS_MS 一致 */

    function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

    /* 键盘 -> 动作 */
    var KEYMAP = {
        KeyW: 'thrUp', KeyS: 'thrDown',
        KeyA: 'yawLeft', KeyD: 'yawRight',
        ArrowUp: 'pitFwd', ArrowDown: 'pitBack',
        ArrowLeft: 'rolLeft', ArrowRight: 'rolRight',
        KeyQ: 'btnLeftTop', KeyE: 'btnRightTop',
        KeyZ: 'trimRolDown', KeyX: 'trimRolUp',
        KeyC: 'trimPitUp', KeyV: 'trimPitDown',
        Space: 'armAuto',
        KeyR: 'reset',
        KeyJ: 'toggleJam',
        KeyH: 'toggleHud',
        Digit1: 'cam0', Digit2: 'cam1', Digit3: 'cam2',
        KeyT: 'timeScale'
    };

    /* 按键事件名 -> 遥控器上的哪个键 */
    var BUTTONS = {
        btnLeftTop: { name: '左上键', short: 'powerDown', long: 'none' },
        btnRightTop: { name: '右上键', short: 'fixHeight', long: 'calibrate' }
    };

    function makeInput() {
        var cfg = QC.config;
        var toggles = { pulse: {} };
        var held = {};          /* 键盘按住状态 */
        var pinned = {};        /* 被外部锁定的通道: 期间不参与键盘积分与回中 */
        var btnDownAt = {};     /* 遥控器按键按下时刻 */

        var input = {
            /* 摇杆行程量(0~1000) */
            axes: { THR: 0, YAW: 500, PIT: 500, ROL: 500 },
            /* 微调量(打到遥控器零偏上) */
            trim: { PIT: 0, ROL: 0 },
            /* 按键产生的一次性命令, 每帧被取走 */
            cmd: {
                isPowerDown: 0, isFixHeight: 0,
                calibrate: 0, reset: 0, toggleJam: 0,
                toggleHud: 0, cam: -1, timeScale: 0, armAuto: 0
            },
            lastKeyName: '',
            events: [],

            /* 虚拟遥控器的摇杆位置(归一化 -1..1), 供 HUD 显示 */
            stickView: { leftX: 0, leftY: -1, rightX: 0, rightY: 0 },

            update: function (dtMs) {
                var fine = !!held.ShiftLeft || !!held.ShiftRight;
                var rate = (fine ? AXIS_RATE_FINE : AXIS_RATE) * dtMs / 1000;
                var a = input.axes;
                var selfCenter = (fine ? 1.6 : 1.0);

                /* 油门: 松手保持 */
                if (!pinned.THR) {
                    if (held.thrUp) a.THR += rate;
                    if (held.thrDown) a.THR -= rate;
                    a.THR = clamp(a.THR, 0, 1000);
                }

                /* 偏航/俯仰/横滚: 松手回中 */
                if (!pinned.YAW) {
                    if (held.yawLeft) a.YAW -= rate;
                    else if (held.yawRight) a.YAW += rate;
                    else a.YAW = approach(a.YAW, 500, rate * selfCenter);
                }
                if (!pinned.PIT) {
                    if (held.pitFwd) a.PIT += rate;
                    else if (held.pitBack) a.PIT -= rate;
                    else a.PIT = approach(a.PIT, 500, rate * selfCenter);
                }
                if (!pinned.ROL) {
                    if (held.rolLeft) a.ROL -= rate;
                    else if (held.rolRight) a.ROL += rate;
                    else a.ROL = approach(a.ROL, 500, rate * selfCenter);
                }

                /* 虚拟遥控器拖动会直接写 axes, 这里同步一下视图 */
                input.stickView.leftX = (a.YAW - 500) / 500;
                input.stickView.leftY = -(a.THR - 500) / 500;
                input.stickView.rightX = (a.ROL - 500) / 500;
                input.stickView.rightY = -(a.PIT - 500) / 500;
            },

            /* 锁定某个通道(自动化演示/测试用): 锁定期间键盘不再改动它 */
            pinAxis: function (name, v) {
                if (v === null || v === undefined) { delete pinned[name]; return; }
                pinned[name] = true;
                input.axes[name] = clamp(Math.round(v), 0, 1000);
            },
            unpinAll: function () { pinned = {}; },

            /* 把摇杆行程量写进遥控器状态(含极性映射) */
            applyToRemote: function (remote, dtMs) {
                var pol = cfg.STICK_POLARITY;
                var a = input.axes;
                function pol_(v, s) { return s < 0 ? (1000 - v) : v; }

                remote.joyStick.THR = Math.round(pol_(clamp(a.THR, 0, 1000), pol.THR));
                remote.joyStick.YAW = Math.round(pol_(clamp(a.YAW, 0, 1000), pol.YAW));
                remote.joyStick.PIT = Math.round(pol_(clamp(a.PIT, 0, 1000), pol.PIT));
                remote.joyStick.ROL = Math.round(pol_(clamp(a.ROL, 0, 1000), pol.ROL));

                /* 微调量叠加到遥控器零偏上(与 App_DataProcess 的微调键一致) */
                remote.joyStickBias.PIT = input.trim.PIT;
                remote.joyStickBias.ROL = input.trim.ROL;

                /* 一次性命令 */
                if (input.cmd.isPowerDown) {
                    remote.joyStick.isPowerDown = 1;
                    input.cmd.isPowerDown = 0;
                }
                if (input.cmd.isFixHeight) {
                    remote.joyStick.isFixHeight = 1;
                    input.cmd.isFixHeight = 0;
                }
            },

            keyName: function (code) { return KEYMAP[code] || ''; },
            isHeld: function (name) { return !!held[name]; },

            /* 键盘事件 */
            onKeyDown: function (e) {
                if (e.repeat) { return; }
                var action = KEYMAP[e.code];
                if (!action) return;
                e.preventDefault();

                if (BUTTONS[action]) {
                    held[action] = true;
                    btnDownAt[action] = performance.now();
                    return;
                }

                switch (action) {
                    case 'armAuto': input.cmd.armAuto = 1; break;
                    case 'reset': input.cmd.reset = 1; break;
                    case 'toggleJam': input.cmd.toggleJam = 1; break;
                    case 'toggleHud': input.cmd.toggleHud = 1; break;
                    case 'timeScale': input.cmd.timeScale = 1; break;
                    case 'cam0': input.cmd.cam = 0; break;
                    case 'cam1': input.cmd.cam = 1; break;
                    case 'cam2': input.cmd.cam = 2; break;
                    case 'trimRolUp': input.trim.ROL -= cfg.TRIM_STEP; break;
                    case 'trimRolDown': input.trim.ROL += cfg.TRIM_STEP; break;
                    case 'trimPitUp': input.trim.PIT -= cfg.TRIM_STEP; break;
                    case 'trimPitDown': input.trim.PIT += cfg.TRIM_STEP; break;
                    default: held[action] = true;
                }
            },

            onKeyUp: function (e) {
                var action = KEYMAP[e.code];
                if (!action) return;
                if (BUTTONS[action]) {
                    held[action] = false;
                    var dur = performance.now() - (btnDownAt[action] || 0);
                    var isLong = dur > LONG_PRESS_MS;
                    var b = BUTTONS[action];
                    var what = isLong ? b.long : b.short;
                    input.events.push(b.name + (isLong ? ' 长按' : ' 短按') +
                        ' (' + Math.round(dur) + 'ms) -> ' + what);
                    if (what === 'powerDown') input.cmd.isPowerDown = 1;
                    if (what === 'fixHeight') input.cmd.isFixHeight = 1;
                    if (what === 'calibrate') input.cmd.calibrate = 1;
                    return;
                }
                held[action] = false;
            },

            /* 供虚拟遥控器拖拽使用: 直接设定某个通道 */
            setAxis: function (name, v) {
                input.axes[name] = clamp(Math.round(v), 0, 1000);
            }
        };

        return input;
    }

    function approach(cur, target, step) {
        if (cur < target) return Math.min(target, cur + step);
        if (cur > target) return Math.max(target, cur - step);
        return cur;
    }

    /**
     * 绑定虚拟遥控器: 两个摇杆区域, 鼠标/触摸拖动
     * 左摇杆 = 油门(上下) + 偏航(左右)
     * 右摇杆 = 俯仰(上下) + 横滚(左右)
     * 这就是遥控器常见的 Mode 2 布局
     */
    function bindVirtualSticks(input, doc) {
        var defs = [
            { id: 'stickLeft', vert: 'THR', horiz: 'YAW', vertUp: +1, horizRight: +1, defVert: 0 },
            { id: 'stickRight', vert: 'PIT', horiz: 'ROL', vertUp: +1, horizRight: +1, defVert: 500 }
        ];

        defs.forEach(function (d) {
            var el = doc.getElementById(d.id);
            if (!el) return;
            var knob = el.querySelector('.knob');
            var dragging = false;

            function apply(ev) {
                var r = el.getBoundingClientRect();
                var nx = ((ev.clientX - r.left) / r.width) * 2 - 1;
                var ny = ((ev.clientY - r.top) / r.height) * 2 - 1;
                nx = clamp(nx, -1, 1);
                ny = clamp(ny, -1, 1);

                /* 水平 */
                var h = 500 + nx * 500 * d.horizRight;
                input.setAxis(d.horiz, h);
                /* 垂直: 屏幕向下为正, 摇杆向上要让行程量增大 */
                var v = 500 - ny * 500 * d.vertUp;
                if (d.vert === 'THR') {
                    /* 油门: -1(最下) -> 0, +1(最上) -> 1000 */
                    v = (1 - ny) / 2 * 1000;
                }
                input.setAxis(d.vert, v);

                if (knob) {
                    knob.style.left = ((nx + 1) / 2 * 100) + '%';
                    knob.style.top = ((ny + 1) / 2 * 100) + '%';
                }
            }

            el.addEventListener('pointerdown', function (ev) {
                dragging = true;
                el.setPointerCapture(ev.pointerId);
                apply(ev);
            });
            el.addEventListener('pointermove', function (ev) {
                if (dragging) apply(ev);
            });
            el.addEventListener('pointerup', function (ev) {
                dragging = false;
                try { el.releasePointerCapture(ev.pointerId); } catch (e) { /* 忽略 */ }
                /* 松手后除油门外的通道回中 */
                if (d.horiz) input.setAxis(d.horiz, 500);
                if (d.vert !== 'THR') input.setAxis(d.vert, 500);
                if (knob) {
                    knob.style.left = '50%';
                    knob.style.top = (d.vert === 'THR') ? knob.style.top : '50%';
                }
            });
            el.addEventListener('pointercancel', function () { dragging = false; });
        });
    }

    QC.input = {
        makeInput: makeInput,
        bindVirtualSticks: bindVirtualSticks,
        BUTTONS: BUTTONS,
        KEYMAP: KEYMAP
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
