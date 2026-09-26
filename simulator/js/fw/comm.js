/*
 * fw/comm.js — 通信协议、连接判定与解锁状态机
 *
 * 对应固件:
 *   遥控器侧: App_Communication_SendJoyStickData   (组帧)
 *   飞控板侧: App_Communication_ReceiveJoyStickData (三级校验 + 解析)
 *             App_Communication_CheckConnection      (连接/失联判定)
 *             App_Communication_RemoteUnlock         (油门序列解锁状态机)
 *
 * 遥控器"只发"、飞控"只收", 两边之间只有 18 字节的帧, 仿真里也是这个接口。
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});
    var cfg = QC.config;

    var Com_OK = 0;
    var Com_TIMEOUT = 1;
    var Com_FAIL = 2;

    /* 油门状态机 (Com_RemoteStatus) */
    var THR_FREE = 0, THR_MAX = 1, THR_MAX_LEAVE = 2, THR_MIN = 3, THR_UNLOCK = 4;

    function Checksum(buf, len) {
        var sum = 0;
        for (var i = 0; i < len; i++) sum += buf[i];
        return sum >>> 0;
    }

    /**
     * 遥控器组帧 (App_Communication_SendJoyStickData)
     * 返回 18 字节的 Uint8Array
     */
    function BuildFrame(remote) {
        var buf = new Uint8Array(cfg.RF_FRAME_LEN);
        var js = remote.joyStick;
        var i = 0;

        buf[i++] = cfg.FRAME_0;
        buf[i++] = cfg.FRAME_1;
        buf[i++] = cfg.FRAME_2;
        buf[i++] = 0; /* 载荷长度, 稍后回填 */

        var thr = js.THR & 0xFFFF;
        var yaw = js.YAW & 0xFFFF;
        var pit = js.PIT & 0xFFFF;
        var rol = js.ROL & 0xFFFF;

        buf[i++] = (thr >> 8) & 0xFF; buf[i++] = thr & 0xFF;
        buf[i++] = (yaw >> 8) & 0xFF; buf[i++] = yaw & 0xFF;
        buf[i++] = (pit >> 8) & 0xFF; buf[i++] = pit & 0xFF;
        buf[i++] = (rol >> 8) & 0xFF; buf[i++] = rol & 0xFF;

        /* 关机/定高都是单次触发, 发一次就清零 */
        buf[i++] = js.isPowerDown;
        js.isPowerDown = 0;
        buf[i++] = js.isFixHeight;
        js.isFixHeight = 0;

        buf[3] = i - cfg.RF_HEADER_LEN;

        var sum = Checksum(buf, i);
        buf[i++] = (sum >>> 24) & 0xFF;
        buf[i++] = (sum >>> 16) & 0xFF;
        buf[i++] = (sum >>> 8) & 0xFF;
        buf[i++] = sum & 0xFF;

        return buf;
    }

    /**
     * 飞控接收解析 (App_Communication_ReceiveJoyStickData)
     * 三级校验: 帧头 -> 载荷长度 -> 累加校验和
     * @param {Uint8Array} rx 收到的 18 字节
     * @returns {number} Com_OK(0) 表示收到有效数据
     */
    function ReceiveFrame(s, rx) {
        /* 2. 校验帧头 */
        if (rx[0] !== cfg.FRAME_0 || rx[1] !== cfg.FRAME_1 || rx[2] !== cfg.FRAME_2) {
            return Com_FAIL;
        }
        /* 3. 校验载荷长度 */
        if (rx[3] !== cfg.RF_PAYLOAD_LEN) return Com_FAIL;

        /* 4. 校验累加和 */
        var sum = Checksum(rx, cfg.RF_HEADER_LEN + cfg.RF_PAYLOAD_LEN);
        var h = cfg.RF_HEADER_LEN + cfg.RF_PAYLOAD_LEN;
        var frameSum = ((rx[h] << 24) | (rx[h + 1] << 16) | (rx[h + 2] << 8) | rx[h + 3]) >>> 0;
        if (sum !== frameSum) return Com_FAIL;

        /* 5. 解析载荷 */
        var p = cfg.RF_HEADER_LEN;
        s.joyStick.THR = (rx[p] << 8) | rx[p + 1];
        s.joyStick.YAW = (rx[p + 2] << 8) | rx[p + 3];
        s.joyStick.PIT = (rx[p + 4] << 8) | rx[p + 5];
        s.joyStick.ROL = (rx[p + 6] << 8) | rx[p + 7];

        s.joyStick.isPowerDown = rx[p + 8];

        if (rx[p + 9]) {
            s.joyStick.isFixHeight = s.joyStick.isFixHeight ? 0 : 1;
            s.isFixHeight = (s.joyStick.isFixHeight === 1) ? Com_OK : Com_FAIL;
        }

        return Com_OK;
    }

    /**
     * 连接判定 (App_Communication_CheckConnection)
     * 收到一帧即判为已连接; 连续 LOST_FRAME_LIMIT 次没收到判为失联
     */
    function CheckConnection(s, isReceiveData) {
        var C = s.comm;

        if (isReceiveData === Com_OK) {
            C.lostCnt = 0;
            return Com_OK;
        }

        if (C.lostCnt < cfg.LOST_FRAME_LIMIT) C.lostCnt++;

        return (C.lostCnt >= cfg.LOST_FRAME_LIMIT) ? Com_FAIL : Com_OK;
    }

    /**
     * 解锁状态机 (App_Communication_RemoteUnlock)
     * 油门推到最大保持 1.2s -> 拉到最小保持 1.2s -> 解锁
     */
    function RemoteUnlock(s, isRemoteConnected) {
        var C = s.comm;
        var thr = s.joyStick.THR;

        /* 1. 失联直接上锁并退出定高 */
        if (isRemoteConnected !== Com_OK) {
            C.remoteStatus = THR_FREE;
            C.thrMaxDuration = 0;
            C.thrMinDuration = 0;
            s.joyStick.isFixHeight = 0;
            s.isFixHeight = Com_FAIL;
            return Com_FAIL;
        }

        switch (C.remoteStatus) {
            case THR_FREE:
                C.thrMaxDuration = 0;
                C.thrMinDuration = 0;
                if (thr >= cfg.THR_NEAR_MAX) C.remoteStatus = THR_MAX;
                break;

            case THR_MAX:
                if (thr >= cfg.THR_NEAR_MAX) {
                    C.thrMaxDuration++;
                    if (C.thrMaxDuration >= cfg.UNLOCK_HOLD_COUNT) C.remoteStatus = THR_MAX_LEAVE;
                } else {
                    C.remoteStatus = THR_FREE;
                }
                break;

            case THR_MAX_LEAVE:
                if (thr <= cfg.THR_NEAR_MIN) C.remoteStatus = THR_MIN;
                break;

            case THR_MIN:
                if (thr <= cfg.THR_NEAR_MIN) {
                    C.thrMinDuration++;
                    if (C.thrMinDuration >= cfg.UNLOCK_HOLD_COUNT) C.remoteStatus = THR_UNLOCK;
                } else {
                    C.remoteStatus = THR_FREE;
                }
                break;

            case THR_UNLOCK:
                if (thr <= cfg.THR_NEAR_MIN) {
                    C.lowDuration++;
                    if (C.lowDuration >= cfg.AUTO_LOCK_COUNT) {
                        C.remoteStatus = THR_FREE;
                        C.lowDuration = 0;
                        s.joyStick.isFixHeight = 0;
                        s.isFixHeight = Com_FAIL;
                        return Com_FAIL;
                    }
                } else {
                    C.lowDuration = 0;
                }
                return Com_OK;

            default:
                break;
        }

        /* 除"已解锁"外都视为未解锁, 且不允许定高 */
        s.joyStick.isFixHeight = 0;
        s.isFixHeight = Com_FAIL;
        return Com_FAIL;
    }

    /**
     * 2.4G 链路模型 (仿真引入)
     * 距离越远、越是被楼体遮挡, 丢包率越高; 帧要么完整到达, 要么完全丢失。
     * 另外有一定概率发生比特错误 —— 此时帧会到达但通不过校验和,
     * 用来验证接收端的三级校验确实能挡住脏数据。
     */
    function LinkModel() {
        return {
            jammed: false,        /* 手动干扰开关 */
            maxRange: 260,        /* m, 超过这个距离基本收不到 */
            corruptProb: 0.004    /* 帧到达但比特错误的比例 */
        };
    }

    /**
     * 模拟一次空口传输
     * @returns {Uint8Array|null} 校验失败或丢失时返回 null
     */
    function Transmit(link, frame, distance, occluded, rng) {
        if (link.jammed) return null;

        /* 距离衰减: 超过 60% 量程后丢包率快速上升 */
        var d = distance / link.maxRange;
        var loss = d > 0.6 ? Math.min(0.95, (d - 0.6) * 2.4) : 0;
        if (occluded) loss = Math.min(0.95, loss + 0.35);

        if (rng() < loss) return null;

        var out = frame.slice();
        if (rng() < link.corruptProb) {
            /* 随机翻转载荷区的一个 bit, 制造一个真实存在的错帧 */
            var idx = cfg.RF_HEADER_LEN + Math.floor(rng() * cfg.RF_PAYLOAD_LEN);
            out[idx] ^= (1 << Math.floor(rng() * 8));
        }
        return out;
    }

    QC.comm = {
        Com_OK: Com_OK,
        Com_TIMEOUT: Com_TIMEOUT,
        Com_FAIL: Com_FAIL,
        THR_FREE: THR_FREE,
        THR_MAX: THR_MAX,
        THR_MAX_LEAVE: THR_MAX_LEAVE,
        THR_MIN: THR_MIN,
        THR_UNLOCK: THR_UNLOCK,
        Checksum: Checksum,
        BuildFrame: BuildFrame,
        ReceiveFrame: ReceiveFrame,
        CheckConnection: CheckConnection,
        RemoteUnlock: RemoteUnlock,
        LinkModel: LinkModel,
        Transmit: Transmit
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
