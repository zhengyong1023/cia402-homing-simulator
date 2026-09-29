// ============================================================
// CiA 402 回零模拟器 — 回零逻辑层
// 纯逻辑代码，无 DOM 依赖
// 依赖外部函数（由 UI 层定义）：log, setState, completeHoming, triggerError
// ============================================================

// === 轴配置 ===
const AXIS_MIN = 0;
const AXIS_MAX = 1000;
const HOME_SWITCH_CENTER = 500;
const HOME_SWITCH_HALF_WIDTH = 43;
const NOT_POS = 10;
const POT_POS = 990;
const INDEX_INTERVAL = 40;
const ACCEL = 300;
const DECEL = 500;

// === 状态 ===
let position = 0;
let velocity = 0;
let targetVelocity = 0;
let currentDirection = 0;
let isRunning = false;
let homingComplete = false;
let currentMode = 1;
let homingState = 'IDLE';
let subState = '';
let lastTimestamp = 0;

let sensorNOT = false;
let sensorHOME = false;
let sensorPOT = false;
let sensorINDEX = false;
let sensorSTALL = false;

let prevSensorHOME = false;
let prevSensorNOT = false;
let prevSensorPOT = false;
let prevSensorINDEX = false;
let prevSensorSTALL = false;

let latchedHomePos = 0;
let latchedIndexPos = 0;
let latchedLimitPos = 0;

// === 回零状态机 ===
let sm = {};

// === 工具函数 ===
function getRisingEdge(curr, prev) { return curr && !prev; }
function getFallingEdge(curr, prev) { return !curr && prev; }

// 查找当前位置之后的第一个Z相信号位置（正向运动时）
function getNextIndexPosition(fromPos) {
  const nextIdx = Math.ceil(fromPos / INDEX_INTERVAL) * INDEX_INTERVAL;
  return nextIdx <= fromPos ? nextIdx + INDEX_INTERVAL : nextIdx;
}

// === 运动控制 ===
function setMotor(direction, vel) {
  currentDirection = direction;
  targetVelocity = vel * direction;
}

function stopMotor() {
  currentDirection = 0;
  targetVelocity = 0;
}

// === 传感器 ===
function updateSensors() {
  prevSensorHOME = sensorHOME;
  prevSensorNOT = sensorNOT;
  prevSensorPOT = sensorPOT;
  prevSensorINDEX = sensorINDEX;
  prevSensorSTALL = sensorSTALL;

  const homeLeft = HOME_SWITCH_CENTER - HOME_SWITCH_HALF_WIDTH;
  const homeRight = HOME_SWITCH_CENTER + HOME_SWITCH_HALF_WIDTH;
  sensorHOME = (position >= homeLeft && position <= homeRight);
  // 信号触发 = 运动机构边缘触碰开关边缘 (motor半宽24u + switch半宽10u = 34u)
  sensorNOT = (position <= NOT_POS + 34);
  sensorPOT = (position >= POT_POS - 34);
  const idxMod = position % INDEX_INTERVAL;
  sensorINDEX = (idxMod <= 5 || idxMod >= INDEX_INTERVAL - 5);
  sensorSTALL = ((position <= AXIS_MIN + 1 || position >= AXIS_MAX - 1) && Math.abs(velocity) < 1);
}

// === 物理引擎 ===
function updatePhysics(dt) {
  if (!isRunning) {
    if (Math.abs(velocity) > 0.1) {
      const decelAmount = DECEL * dt;
      if (velocity > 0) velocity = Math.max(0, velocity - decelAmount);
      else velocity = Math.min(0, velocity + decelAmount);
    } else {
      velocity = 0;
    }
  } else {
    if (velocity < targetVelocity) {
      velocity = Math.min(targetVelocity, velocity + ACCEL * dt);
    } else if (velocity > targetVelocity) {
      velocity = Math.max(targetVelocity, velocity - ACCEL * dt);
    }
  }
  position += velocity * dt;
  if (position <= AXIS_MIN) { position = AXIS_MIN; if (velocity < 0) velocity = 0; }
  if (position >= AXIS_MAX) { position = AXIS_MAX; if (velocity > 0) velocity = 0; }
}

// === 状态机初始化 ===
function initStateMachine(mode, highVel, lowVel) {
  sm = { phase: 0, velocity: highVel || 80, lowVel: lowVel || 20, dir: 0, wasStalled: false };
  sensorSTALL = false;
  prevSensorSTALL = false;
  // 初始化传感器，确保 prevSensor* 与当前位置一致，避免首帧边缘检测误触发
  updateSensors();
  prevSensorNOT = sensorNOT;
  prevSensorHOME = sensorHOME;
  prevSensorPOT = sensorPOT;
  prevSensorINDEX = sensorINDEX;
}

// === 状态机调度 ===
function tickStateMachine(dt) {
  if (homingComplete || !isRunning) return;
  // updateSensors() 由 mainLoop 在每帧初调用一次，此处不应重复调用
  // 否则 prevSensor* 会被覆盖为当前值，导致边缘检测失效
  const m = currentMode;
  if (m <= -1 && m >= -6) tickStallMethod(m, dt);
  else if (m === 1) tickMethod1(dt);
  else if (m === 2) tickMethod2(dt);
  else if (m === 3) tickMethod3(dt);
  else if (m === 4) tickMethod4(dt);
  else if (m === 5) tickMethod5(dt);
  else if (m === 6) tickMethod6(dt);
  else if (m === 7) tickMethod7(dt);
  else if (m === 8) tickMethod8(dt);
  else if (m === 9) tickMethod9(dt);
  else if (m === 10) tickMethod10(dt);
  else if (m === 11) tickMethod11(dt);
  else if (m === 12) tickMethod12(dt);
  else if (m === 13) tickMethod13(dt);
  else if (m === 14) tickMethod14(dt);
  else if (m === 17) tickMethod17(dt);
  else if (m === 18) tickMethod18(dt);
  else if (m === 19) tickMethod19(dt);
  else if (m === 20) tickMethod20(dt);
  else if (m === 21) tickMethod21(dt);
  else if (m === 22) tickMethod22(dt);
  else if (m === 23) tickMethod23(dt);
  else if (m === 24) tickMethod24(dt);
  else if (m === 25) tickMethod25(dt);
  else if (m === 26) tickMethod26(dt);
  else if (m === 27) tickMethod27(dt);
  else if (m === 28) tickMethod28(dt);
  else if (m === 29) tickMethod29(dt);
  else if (m === 30) tickMethod30(dt);
  else if (m === 33) tickMethod33(dt);
  else if (m === 34) tickMethod34(dt);
  else if (m === 35) tickMethod35(dt);
}

// ============================================================
// 堵转回零方法 -6, -5, -4, -3, -2, -1
// ============================================================
function tickStallMethod(method, dt) {
  switch(sm.phase) {
    case 0:
      // 开始运动，等待堵转
      if (method === -6) {
        setState('SEEKING', 'Moving negative (Low Speed)');
        setMotor(-1, sm.lowVel);
      } else if (method === -5) {
        setState('SEEKING', 'Moving positive (Low Speed)');
        setMotor(1, sm.lowVel);
      } else if (method === -4) {
        setState('SEEKING', 'Moving negative (High Speed)');
        setMotor(-1, sm.velocity);
      } else if (method === -3) {
        setState('SEEKING', 'Moving positive (High Speed)');
        setMotor(1, sm.velocity);
      } else if (method === -2) {
        setState('SEEKING', 'Moving negative (High Speed)');
        setMotor(-1, sm.velocity);
      } else if (method === -1) {
        setState('SEEKING', 'Moving positive (High Speed)');
        setMotor(1, sm.velocity);
      }
      sm.phase = 1;
      break;

    case 1:
      // 等待堵转信号触发
      if (sensorSTALL) {
        sm.wasStalled = true;
        log('方法 ' + method + '：堵转检测到于 ' + position.toFixed(1));
        
        if (method === -6 || method === -5) {
          stopMotor();
          completeHoming('堵转位置 ' + position.toFixed(1));
          return;
        }
        
        // 高速堵转，先停止电机
        stopMotor();
        sm.timer = 0.5; // 💡 新增：加入 0.5 秒的堵转保持延时，用于 UI 展示和真实感模拟
        sm.phase = 2;
      }
      break;

    case 2:
      // 等待电机完全停止，并保持一段时间让堵转信号清晰展示
      if (Math.abs(velocity) < 0.5) {
        sm.timer -= dt; // 💡 倒计时
        setState('STALLED', 'Holding stall to confirm'); // 更新面板状态
        
        if (sm.timer <= 0) {
          setState('REVERSING', 'Reversing to release torque');
          if (method === -4 || method === -2) {
            setMotor(1, sm.lowVel);
          } else if (method === -3 || method === -1) {
            setMotor(-1, sm.lowVel);
          }
          sm.phase = 3;
        }
      }
      break;

    case 3:
      // 等待转矩消失
      // 要求电机物理位置必须实质性脱离死挡块区域（离开两端极限至少 2 个单位）
      const isPhysicallyClear = (position > AXIS_MIN + 2 && position < AXIS_MAX - 2);
      
      if (!sensorSTALL && isPhysicallyClear) {
        log('方法 ' + method + '：转矩消失于 ' + position.toFixed(1));
        
        if (method === -4 || method === -3) {
          stopMotor();
          completeHoming('转矩消失位置 ' + position.toFixed(1));
        } else if (method === -2 || method === -1) {
          sm.phase = 4;
          setState('INDEX SEARCH', 'Seeking Z-phase');
        }
      }
      break;

    case 4:
      // 寻找Z相
      if (sensorINDEX && getRisingEdge(sensorINDEX, prevSensorINDEX)) {
        log('方法 ' + method + '：Index脉冲发现于 ' + position.toFixed(1));
        completeHoming('Z相位置 ' + position.toFixed(1));
      }
      break;
  }
}

// ============================================================
// 方法1：负限位上升沿 + Z相
// A情况：负向高速→负限位有效停止→正向低速→离开后第一个Z相
// B情况：已在负限位上→正向高速→负限位无效停止→执行A情况
// 异常：正限位有效，立即停止，回零失败
// ============================================================
function tickMethod1(dt) {
  switch(sm.phase) {
    case 0:
      // 异常拦截
      if (sensorPOT) { triggerError('正限位有效，回零失败'); return; }
      
      if (sensorNOT) {
        // B情况：初始已在负限位上
        setState('MOVING', 'Already at negative limit, moving positive');
        setMotor(1, sm.velocity); // 正向高速退出
        sm.phase = 10;
        log('方法 1(B)：已在负限位上，正向退出');
      } else {
        // A情况：初始不在负限位上
        setState('SEEKING', 'Moving negative');
        setMotor(-1, sm.velocity); // 负向高速寻找
        sm.phase = 1;
      }
      break;

    // === A情况流程 ===
    case 1:
      setMotor(-1, sm.velocity);
      if (sensorPOT) { triggerError('运动中正限位触发'); return; }
      if (sensorNOT) {
        stopMotor();
        sm.phase = 2;
        log('方法 1(A)：负限位触发于 ' + position.toFixed(1) + '，减速停止');
      }
      break;
    case 2:
      // 等待停止后正向低速脱离
      if (Math.abs(velocity) < 0.5) {
        setState('BACKING OFF', 'Leaving limit');
        setMotor(1, sm.lowVel);
        sm.phase = 3;
      }
      break;
    case 3:
      setMotor(1, sm.lowVel);
      if (sensorPOT) { triggerError('运动中正限位触发'); return; }
      if (!sensorNOT) {
        sm.phase = 4;
        log('方法 1：已离开负限位，搜索Z相');
      }
      break;
    case 4:
      setMotor(1, sm.lowVel);
      if (sensorPOT) { triggerError('运动中正限位触发'); return; }
      // 离开限位后，检测到Z相上升沿即停止
      if (sensorINDEX && getRisingEdge(sensorINDEX, prevSensorINDEX)) {
        log('方法 1：Index脉冲发现于 ' + position.toFixed(1));
        completeHoming('Z相位置 ' + position.toFixed(1));
      }
      break;

    // === B情况流程 ===
    case 10:
      setMotor(1, sm.velocity);
      if (sensorPOT) { triggerError('运动中正限位触发'); return; }
      // 等待负限位信号无效
      if (!sensorNOT) {
        stopMotor();
        sm.phase = 11;
        log('方法 1(B)：负限位信号无效于 ' + position.toFixed(1) + '，停止后将执行A情况');
      }
      break;
    case 11:
      // 等待电机完全停止后，执行A情况
      if (Math.abs(velocity) < 0.5) {
        setState('SEEKING', 'Moving negative (Condition A)');
        setMotor(-1, sm.velocity); // 再次以负向高速寻找限位
        sm.phase = 1; // 严丝合缝地对接到 A情况 的 Phase 1
      }
      break;
  }
}

// ============================================================
// 方法2：正限位上升沿 + Z相（与方法1完全对称）
// A情况：正向高速→正限位有效停止→负向低速→离开后第一个Z相
// B情况：已在正限位上→负向高速→正限位无效停止→执行A情况
// 异常：负限位有效，立即停止，回零失败
// ============================================================
function tickMethod2(dt) {
  switch(sm.phase) {
    case 0:
      // 异常拦截
      if (sensorNOT) { triggerError('负限位有效，回零失败'); return; }
      
      if (sensorPOT) {
        // B情况：初始已在正限位上
        setState('MOVING', 'Already at positive limit, moving negative');
        setMotor(-1, sm.velocity); // 负向高速退出
        sm.phase = 10;
        log('方法 2(B)：已在正限位上，负向退出');
      } else {
        // A情况：初始不在正限位上
        setState('SEEKING', 'Moving positive');
        setMotor(1, sm.velocity); // 正向高速寻找
        sm.phase = 1;
      }
      break;

    // === A情况流程 ===
    case 1:
      setMotor(1, sm.velocity);
      if (sensorNOT) { triggerError('运动中负限位触发'); return; }
      if (sensorPOT) {
        stopMotor();
        sm.phase = 2;
        log('方法 2(A)：正限位触发于 ' + position.toFixed(1) + '，减速停止');
      }
      break;
    case 2:
      // 等待停止后负向低速脱离
      if (Math.abs(velocity) < 0.5) {
        setState('BACKING OFF', 'Leaving limit');
        setMotor(-1, sm.lowVel);
        sm.phase = 3;
      }
      break;
    case 3:
      setMotor(-1, sm.lowVel);
      if (sensorNOT) { triggerError('运动中负限位触发'); return; }
      if (!sensorPOT) {
        sm.phase = 4;
        log('方法 2：已离开正限位，搜索Z相');
      }
      break;
    case 4:
      setMotor(-1, sm.lowVel);
      if (sensorNOT) { triggerError('运动中负限位触发'); return; }
      // 离开限位后，检测到Z相上升沿即停止
      if (sensorINDEX && getRisingEdge(sensorINDEX, prevSensorINDEX)) {
        log('方法 2：Index脉冲发现于 ' + position.toFixed(1));
        completeHoming('Z相位置 ' + position.toFixed(1));
      }
      break;

    // === B情况流程 ===
    case 10:
      setMotor(-1, sm.velocity);
      if (sensorNOT) { triggerError('运动中负限位触发'); return; }
      // 等待正限位信号无效
      if (!sensorPOT) {
        stopMotor();
        sm.phase = 11;
        log('方法 2(B)：正限位信号无效于 ' + position.toFixed(1) + '，停止后将执行A情况');
      }
      break;
    case 11:
      // 等待电机完全停止后，执行A情况
      if (Math.abs(velocity) < 0.5) {
        setState('SEEKING', 'Moving positive (Condition A)');
        setMotor(1, sm.velocity); // 再次以正向高速寻找限位
        sm.phase = 1; // 对接 A情况 的 Phase 1
      }
      break;
  }
}

// ============================================================
// 抽象通用方法：针对 3/5 和 4/6 的对称镜像特性
// ============================================================

function tickHomingType3_5(isForward) {
  const dir = isForward ? 1 : -1;
  const method = isForward ? 3 : 5;

  // 统一异常拦截：任何时候触碰任意限位，立即报错退出
  if (sensorNOT || sensorPOT) { triggerError('运动中限位触发，回零失败'); return; }

  switch(sm.phase) {
    case 0: // 初始判定
      if (sensorHOME) {
        setState('MOVING', 'Condition B');
        setMotor(-dir, sm.velocity);
        sm.phase = 10;
      } else {
        setState('SEEKING', 'Condition A');
        setMotor(dir, sm.velocity);
        sm.phase = 1;
      }
      break;
    // === A情况 ===
    case 1:
      setMotor(dir, sm.velocity);
      if (sensorHOME) { stopMotor(); sm.phase = 2; log(`方法 ${method}(A)：原点有效，减速停止`); }
      break;
    case 2:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.lowVel); sm.phase = 3; setState('BACKING OFF', 'Leaving home'); }
      break;
    case 3:
      setMotor(-dir, sm.lowVel);
      if (!sensorHOME) { sm.phase = 4; log(`方法 ${method}：已离开原点，搜索Z相`); }
      break;
    case 4:
      setMotor(-dir, sm.lowVel);
      if (sensorINDEX && getRisingEdge(sensorINDEX, prevSensorINDEX)) {
        completeHoming('Z相位置 ' + position.toFixed(1));
      }
      break;
    // === B情况 ===
    case 10:
      setMotor(-dir, sm.velocity);
      if (!sensorHOME) { stopMotor(); sm.phase = 11; log(`方法 ${method}(B)：已离开原点信号区域`); }
      break;
    case 11:
      if (Math.abs(velocity) < 0.5) { setMotor(dir, sm.velocity); sm.phase = 1; setState('SEEKING', 'Executing Condition A'); }
      break;
  }
}

function tickHomingType4_6(isForward) {
  const dir = isForward ? 1 : -1;
  const method = isForward ? 4 : 6;

  // 统一异常拦截
  if (sensorNOT || sensorPOT) { triggerError('运动中限位触发，回零失败'); return; }

  switch(sm.phase) {
    case 0: // 初始判定
      if (sensorHOME) {
        setState('MOVING', 'Condition B');
        setMotor(-dir, sm.velocity);
        sm.phase = 10;
      } else {
        setState('SEEKING', 'Condition A');
        setMotor(dir, sm.velocity);
        sm.phase = 1;
      }
      break;
    // === A情况 ===
    case 1:
      setMotor(dir, sm.velocity);
      if (sensorHOME) { stopMotor(); sm.phase = 2; log(`方法 ${method}(A)：原点有效，减速停止，转执行B`); }
      break;
    case 2: // A执行完毕，转接B
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 10; setState('MOVING', 'Executing Condition B'); }
      break;
    // === B情况 ===
    case 10:
      setMotor(-dir, sm.velocity);
      if (!sensorHOME) { stopMotor(); sm.phase = 11; log(`方法 ${method}：已离开原点`); }
      break;
    case 11:
      if (Math.abs(velocity) < 0.5) { setMotor(dir, sm.lowVel); sm.phase = 12; setState('INDEX SEARCH', 'Seeking Home and Z'); }
      break;
    case 12:
      setMotor(dir, sm.lowVel);
      // 修复Bug：此处仅记录状态变更，不要 stopMotor()
      if (sensorHOME) { sm.phase = 13; log(`方法 ${method}：原点再次有效，深入原点内搜索Z相`); } 
      break;
    case 13:
      setMotor(dir, sm.lowVel);
      if (sensorINDEX && getRisingEdge(sensorINDEX, prevSensorINDEX)) {
        completeHoming('Z相位置 ' + position.toFixed(1));
      }
      break;
  }
}
// ============================================================
// 方法3-6：原点开关+Z相 状态机路由层
// ============================================================

function tickMethod3(dt) { tickHomingType3_5(true); }
function tickMethod4(dt) { tickHomingType4_6(true); }
function tickMethod5(dt) { tickHomingType3_5(false); }
function tickMethod6(dt) { tickHomingType4_6(false); }
// ============================================================
// 抽象通用方法：针对 7-14 的对称镜像特性
// isForward = true (方法7/8/9/10), false (方法11/12/13/14)
// ============================================================

function tickHomingType7_11(isForward) {
  const dir = isForward ? 1 : -1;
  const limitErr = isForward ? sensorNOT : sensorPOT;
  const limitRev = isForward ? sensorPOT : sensorNOT;
  const method = isForward ? 7 : 11;

  switch(sm.phase) {
    case 0: // 初始判定
      if (limitErr) { triggerError((isForward ? '负' : '正') + '限位有效，回零失败'); return; }
      if (sensorHOME) {
        setState('MOVING', 'Condition B');
        setMotor(-dir, sm.velocity);
        sm.phase = 10;
      } else {
        setState('SEEKING', 'Condition A');
        setMotor(dir, sm.velocity);
        sm.phase = 1;
      }
      break;
    // === A情况 ===
    case 1:
      setMotor(dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { stopMotor(); sm.phase = 2; log(`方法 ${method}(A)：原点有效`); }
      else if (limitRev) { stopMotor(); sm.phase = 20; log(`方法 ${method}(C)：反向限位触发`); }
      break;
    case 2:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.lowVel); sm.phase = 3; setState('BACKING OFF', 'Leaving home'); }
      break;
    case 3:
      setMotor(-dir, sm.lowVel);
      if (limitRev) { triggerError('运动中限位触发'); return; }
      if (!sensorHOME) { sm.phase = 4; log(`方法 ${method}：已离开原点，搜索Z相`); }
      break;
    case 4:
      setMotor(-dir, sm.lowVel);
      if (limitRev) { triggerError('运动中限位触发'); return; }
      if (sensorINDEX && getRisingEdge(sensorINDEX, prevSensorINDEX)) {
        completeHoming('Z相位置 ' + position.toFixed(1));
      }
      break;
    // === B情况 ===
    case 10:
      setMotor(-dir, sm.velocity);
      if (limitRev) { triggerError('运动中限位触发'); return; }
      if (!sensorHOME) { stopMotor(); sm.phase = 11; log(`方法 ${method}(B)：已离开原点`); }
      break;
    case 11:
      if (Math.abs(velocity) < 0.5) { setMotor(dir, sm.velocity); sm.phase = 1; setState('SEEKING', 'Executing Condition A'); }
      break;
    // === C情况 ===
    case 20:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 21; setState('RETURNING', 'Condition C'); }
      break;
    case 21:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { sm.phase = 22; }
      break;
    case 22:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (!sensorHOME) { stopMotor(); sm.phase = 23; log(`方法 ${method}(C)：已穿过原点`); }
      break;
    case 23:
      if (Math.abs(velocity) < 0.5) { setMotor(dir, sm.velocity); sm.phase = 1; setState('SEEKING', 'Executing Condition A'); }
      break;
  }
}

// ============================================================
// 抽象通用方法：针对 8 和 12 的对称镜像特性
// 方法8（正向找原点，原点内找Z相），方法12是对称负向
// ============================================================
function tickHomingType8_12(isForward) {
  const dir = isForward ? 1 : -1;
  const limitErr = isForward ? sensorNOT : sensorPOT;
  const limitRev = isForward ? sensorPOT : sensorNOT;
  const method = isForward ? 8 : 12;

  switch(sm.phase) {
    case 0:
      if (limitErr) { triggerError((isForward ? '负' : '正') + '限位有效，回零失败'); return; }
      if (sensorHOME) {
        setState('MOVING', 'Condition B');
        setMotor(-dir, sm.velocity);
        sm.phase = 11; // 🚨 核心修复：B情况起步已经是停止状态，直接跳过 Phase 10，进入离开原点的监测阶段
      } else {
        setState('SEEKING', 'Condition A');
        setMotor(dir, sm.velocity);
        sm.phase = 1;
      }
      break;
    // === A情况 ===
    case 1:
      setMotor(dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      // A情况：遇到原点，必须先减速停止，所以跳转到 Phase 10 等待停止
      if (sensorHOME) { stopMotor(); sm.phase = 10; log(`方法 ${method}(A)：原点有效，转执行B`); }
      else if (limitRev) { stopMotor(); sm.phase = 20; log(`方法 ${method}(C)：反向限位触发`); }
      break;
    // === A过渡到B的停顿等待 ===
    case 10:
      // 只有A情况撞到原点后才需要在这里等速度降为0，然后再以高速反向退出
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 11; setState('BACKING OFF', 'Leaving home'); }
      break;
    // === B情况（离开原点并停止） ===
    case 11:
      setMotor(-dir, sm.velocity);
      if (limitRev) { triggerError('运动中限位触发'); return; }
      if (!sensorHOME) { 
        stopMotor(); // 🚨 离开原点，下发停止指令
        sm.phase = 12; 
        log(`方法 ${method}：离开原点`); 
      }
      break;
    // === B情况后续（低速找原点和Z相） ===
    case 12:
      // 严格按照时序图：必须等电机完全停稳后，再正向低速运动
      if (Math.abs(velocity) < 0.5) { setMotor(dir, sm.lowVel); sm.phase = 13; setState('INDEX SEARCH', 'Seeking Home and Z'); }
      break;
    case 13:
      setMotor(dir, sm.lowVel);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { sm.phase = 14; log(`方法 ${method}：原点再次有效，搜索Z相`); }
      break;
    case 14:
      setMotor(dir, sm.lowVel);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      // 确保是在原点区域内找到的上升沿 Z 相
      if (sensorINDEX && getRisingEdge(sensorINDEX, prevSensorINDEX)) {
        completeHoming('Z相位置 ' + position.toFixed(1));
      }
      break;
    // === C情况（撞对侧限位后的折返） ===
    case 20:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 21; setState('RETURNING', 'Condition C'); }
      break;
    case 21:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { sm.phase = 22; }
      break;
    case 22:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      // C情况穿过原点后，复用 B情况 的“减速停止后低速找Z”逻辑
      if (!sensorHOME) { stopMotor(); sm.phase = 12; log(`方法 ${method}(C)：穿过原点，转执行B情况低速找Z过程`); }
      break;
  }
}

function tickHomingType9_13(isForward) {
  const dir = isForward ? 1 : -1;
  const limitErr = isForward ? sensorNOT : sensorPOT;
  const limitRev = isForward ? sensorPOT : sensorNOT;
  const method = isForward ? 9 : 13;

  switch(sm.phase) {
    case 0:
      if (limitErr) { triggerError((isForward ? '负' : '正') + '限位有效，回零失败'); return; }
      if (sensorHOME) {
        setState('MOVING', 'Condition B');
        setMotor(dir, sm.velocity);
        sm.phase = 10;
      } else {
        setState('SEEKING', 'Condition A');
        setMotor(dir, sm.velocity);
        sm.phase = 1;
      }
      break;
    // === A情况 ===
    case 1:
      setMotor(dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { sm.phase = 2; log(`方法 ${method}(A)：进入原点`); }
      else if (limitRev) { stopMotor(); sm.phase = 20; log(`方法 ${method}(C)：反向限位触发`); }
      break;
    case 2:
      setMotor(dir, sm.velocity);
      if (limitRev) { stopMotor(); sm.phase = 20; }
      if (!sensorHOME) { stopMotor(); sm.phase = 3; log(`方法 ${method}：已离开原点`); }
      break;
    case 3:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.lowVel); sm.phase = 4; setState('INDEX SEARCH', 'Seeking Home and Z'); }
      break;
    case 4:
      setMotor(-dir, sm.lowVel);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { sm.phase = 5; log(`方法 ${method}：原点有效，搜索Z相`); }
      break;
    case 5:
      setMotor(-dir, sm.lowVel);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorINDEX && getRisingEdge(sensorINDEX, prevSensorINDEX)) {
        completeHoming('Z相位置 ' + position.toFixed(1));
      }
      break;
    // === B情况 ===
    case 10:
      setMotor(dir, sm.velocity);
      if (limitRev) { stopMotor(); sm.phase = 20; }
      if (!sensorHOME) { stopMotor(); sm.phase = 3; log(`方法 ${method}(B)：已离开原点`); }
      break;
    // === C情况 ===
    case 20:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 21; setState('RETURNING', 'Condition C'); }
      break;
    case 21:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { stopMotor(); sm.phase = 22; log(`方法 ${method}(C)：原点有效停止`); }
      break;
    case 22:
      if (Math.abs(velocity) < 0.5) { sm.phase = 10; log(`方法 ${method}(C)：转执行B`); }
      break;
  }
}

function tickHomingType10_14(isForward) {
  const dir = isForward ? 1 : -1;
  const limitErr = isForward ? sensorNOT : sensorPOT;
  const limitRev = isForward ? sensorPOT : sensorNOT;
  const method = isForward ? 10 : 14;

  switch(sm.phase) {
    case 0:
      if (limitErr) { triggerError((isForward ? '负' : '正') + '限位有效，回零失败'); return; }
      if (sensorHOME) {
        setState('MOVING', 'Condition B');
        setMotor(dir, sm.velocity);
        sm.phase = 10; // 跳转进入穿过原点流程
      } else {
        setState('SEEKING', 'Condition A');
        setMotor(dir, sm.velocity);
        sm.phase = 1;
      }
      break;
    // === A情况 ===
    case 1:
      setMotor(dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { sm.phase = 2; log(`方法 ${method}(A)：进入原点`); }
      else if (limitRev) { stopMotor(); sm.phase = 20; log(`方法 ${method}(C)：反向限位触发`); }
      break;
    case 2:
    case 10: // B情况共享穿过原点逻辑
      setMotor(dir, sm.velocity);
      if (limitRev) { stopMotor(); sm.phase = 20; }
      if (!sensorHOME) { stopMotor(); sm.phase = 3; log(`方法 ${method}：穿过原点，减速`); }
      break;
    case 3:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 4; setState('RETURNING', 'Returning to home'); }
      break;
    case 4:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { stopMotor(); sm.phase = 5; log(`方法 ${method}：原点再次有效`); }
      break;
    case 5:
      if (Math.abs(velocity) < 0.5) { setMotor(dir, sm.lowVel); sm.phase = 6; setState('INDEX SEARCH', 'Leaving home for Z'); }
      break;
    case 6:
      setMotor(dir, sm.lowVel);
      if (limitRev) { triggerError('运动中限位触发'); return; }
      if (!sensorHOME) { sm.phase = 7; log(`方法 ${method}：离开原点，找Z相`); }
      break;
    case 7:
      setMotor(dir, sm.lowVel);
      if (limitRev) { triggerError('运动中限位触发'); return; }
      if (sensorINDEX && getRisingEdge(sensorINDEX, prevSensorINDEX)) {
        completeHoming('Z相位置 ' + position.toFixed(1));
      }
      break;
    // === C情况 ===
    case 20:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 21; setState('RETURNING', 'Condition C'); }
      break;
    case 21:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { stopMotor(); sm.phase = 5; log(`方法 ${method}(C)：原点有效停止`); }
      break;
  }
}
// ============================================================
// 方法7-14：限位+原点+Z相 状态机路由层
// ============================================================

function tickMethod7(dt) { tickHomingType7_11(true); }
function tickMethod8(dt) { tickHomingType8_12(true); }
function tickMethod9(dt) { tickHomingType9_13(true); }
function tickMethod10(dt) { tickHomingType10_14(true); }

function tickMethod11(dt) { tickHomingType7_11(false); }
function tickMethod12(dt) { tickHomingType8_12(false); }
function tickMethod13(dt) { tickHomingType9_13(false); }
function tickMethod14(dt) { tickHomingType10_14(false); }


// ============================================================
// 方法17：负限位边缘检测（纯限位，无Z相）——同方法1流程，离开限位即停止
// A情况：负限位无效→负向高速→负限位有效停止→正向低速→离开负限位停止=原点
// B情况：已在负限位上→正向高速→负限位无效停止→执行A情况
// 异常：正限位有效，立即停止，回零失败
// ============================================================
function tickMethod17(dt) {
  switch(sm.phase) {
    case 0:
      // 异常拦截
      if (sensorPOT) { triggerError('正限位有效，回零失败'); return; }

      if (sensorNOT) {
        // B情况：初始已在负限位上
        setState('MOVING', 'Already at negative limit, moving positive');
        setMotor(1, sm.velocity); // 正向高速退出
        sm.phase = 10;
        log('方法 17(B)：已在负限位上，正向退出');
      } else {
        // A情况：初始不在负限位上
        setState('SEEKING', 'Moving negative');
        setMotor(-1, sm.velocity); // 负向高速寻找
        sm.phase = 1;
      }
      break;

    // === A情况流程 ===
    case 1:
      setMotor(-1, sm.velocity);
      if (sensorPOT) { triggerError('运动中正限位触发'); return; }
      if (sensorNOT) {
        stopMotor();
        sm.phase = 2;
        log('方法 17(A)：负限位触发于 ' + position.toFixed(1) + '，减速停止');
      }
      break;
    case 2:
      // 等待停止后正向低速脱离
      if (Math.abs(velocity) < 0.5) {
        setState('BACKING OFF', 'Leaving limit');
        setMotor(1, sm.lowVel);
        sm.phase = 3;
      }
      break;
    case 3:
      setMotor(1, sm.lowVel);
      if (sensorPOT) { triggerError('运动中正限位触发'); return; }
      // 离开负限位瞬间即停止
      if (!sensorNOT) {
        stopMotor(); 
        log('方法 17：已离开负限位于 ' + position.toFixed(1) + ' → 该位置即原点');
        completeHoming('限位边缘位置 ' + position.toFixed(1));
      }
      break;

    // === B情况流程 ===
    case 10:
      setMotor(1, sm.velocity);
      if (sensorPOT) { triggerError('运动中正限位触发'); return; }
      // 等待负限位信号无效
      if (!sensorNOT) {
        stopMotor();
        sm.phase = 11;
        log('方法 17(B)：负限位信号无效于 ' + position.toFixed(1) + '，停止后将执行A情况');
      }
      break;
    case 11:
      // 等待电机完全停止后，执行A情况
      if (Math.abs(velocity) < 0.5) {
        setState('SEEKING', 'Moving negative (Condition A)');
        setMotor(-1, sm.velocity); // 再次以负向高速寻找限位
        sm.phase = 1; // 对接 A情况 的 Phase 1
      }
      break;
  }
}

// ============================================================
// 方法18：正限位边缘检测（纯限位，无Z相）——同方法2流程，离开限位即停止
// A情况：正限位无效→正向高速→正限位有效停止→负向低速→离开正限位停止=原点
// B情况：已在正限位上→负向高速→正限位无效停止→执行A情况
// 异常：负限位有效，立即停止，回零失败
// ============================================================
function tickMethod18(dt) {
  switch(sm.phase) {
    case 0:
      // 异常拦截
      if (sensorNOT) { triggerError('负限位有效，回零失败'); return; }

      if (sensorPOT) {
        // B情况：初始已在正限位上
        setState('MOVING', 'Already at positive limit, moving negative');
        setMotor(-1, sm.velocity); // 负向高速退出
        sm.phase = 10;
        log('方法 18(B)：已在正限位上，负向退出');
      } else {
        // A情况：初始不在正限位上
        setState('SEEKING', 'Moving positive');
        setMotor(1, sm.velocity); // 正向高速寻找
        sm.phase = 1;
      }
      break;

    // === A情况流程 ===
    case 1:
      setMotor(1, sm.velocity);
      if (sensorNOT) { triggerError('运动中负限位触发'); return; }
      if (sensorPOT) {
        stopMotor();
        sm.phase = 2;
        log('方法 18(A)：正限位触发于 ' + position.toFixed(1) + '，减速停止');
      }
      break;
    case 2:
      // 等待停止后负向低速脱离
      if (Math.abs(velocity) < 0.5) {
        setState('BACKING OFF', 'Leaving limit');
        setMotor(-1, sm.lowVel);
        sm.phase = 3;
      }
      break;
    case 3:
      setMotor(-1, sm.lowVel);
      if (sensorNOT) { triggerError('运动中负限位触发'); return; }
      // 离开正限位瞬间即停止
      if (!sensorPOT) {
        stopMotor(); 
        log('方法 18：已离开正限位于 ' + position.toFixed(1) + ' → 该位置即原点');
        completeHoming('限位边缘位置 ' + position.toFixed(1));
      }
      break;

    // === B情况流程 ===
    case 10:
      setMotor(-1, sm.velocity);
      if (sensorNOT) { triggerError('运动中负限位触发'); return; }
      // 等待正限位信号无效
      if (!sensorPOT) {
        stopMotor();
        sm.phase = 11;
        log('方法 18(B)：正限位信号无效于 ' + position.toFixed(1) + '，停止后将执行A情况');
      }
      break;
    case 11:
      // 等待电机完全停止后，执行A情况
      if (Math.abs(velocity) < 0.5) {
        setState('SEEKING', 'Moving positive (Condition A)');
        setMotor(1, sm.velocity); // 再次以正向高速寻找限位
        sm.phase = 1; // 对接 A情况 的 Phase 1
      }
      break;
  }
}
// ============================================================
// 方法19-30 抽象通用方法：原点开关边缘检测（不找Z相）
// 逻辑与 3-14 一致，但在原点边缘变化瞬间直接完成回零
// ============================================================

function tickEdgeType19_21(isForward) {
  const dir = isForward ? 1 : -1;
  const method = isForward ? 19 : 21;
  if (sensorNOT || sensorPOT) { triggerError('运动中限位触发，回零失败'); return; }

  switch(sm.phase) {
    case 0:
      if (sensorHOME) { setState('MOVING', 'Condition B'); setMotor(-dir, sm.velocity); sm.phase = 10; } 
      else { setState('SEEKING', 'Condition A'); setMotor(dir, sm.velocity); sm.phase = 1; }
      break;
    case 1:
      setMotor(dir, sm.velocity);
      if (sensorHOME) { stopMotor(); sm.phase = 2; log(`方法 ${method}(A)：原点有效，减速停止`); }
      break;
    case 2:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.lowVel); sm.phase = 3; setState('BACKING OFF', 'Leaving home'); }
      break;
    case 3:
      setMotor(-dir, sm.lowVel);
      // 核心区别：一离开原点立即停止
      if (!sensorHOME) { stopMotor(); completeHoming('原点边缘位置 ' + position.toFixed(1)); }
      break;
    case 10:
      setMotor(-dir, sm.velocity);
      if (!sensorHOME) { stopMotor(); sm.phase = 11; log(`方法 ${method}(B)：已离开原点`); }
      break;
    case 11:
      if (Math.abs(velocity) < 0.5) { setMotor(dir, sm.velocity); sm.phase = 1; setState('SEEKING', 'Executing Condition A'); }
      break;
  }
}

function tickEdgeType20_22(isForward) {
  const dir = isForward ? 1 : -1;
  const method = isForward ? 20 : 22;
  if (sensorNOT || sensorPOT) { triggerError('运动中限位触发，回零失败'); return; }

  switch(sm.phase) {
    case 0:
      if (sensorHOME) { setState('MOVING', 'Condition B'); setMotor(-dir, sm.velocity); sm.phase = 11; /* B情况直接进入离开监测 */ } 
      else { setState('SEEKING', 'Condition A'); setMotor(dir, sm.velocity); sm.phase = 1; }
      break;
    case 1:
      setMotor(dir, sm.velocity);
      if (sensorHOME) { stopMotor(); sm.phase = 10; log(`方法 ${method}(A)：原点有效，转执行B`); }
      break;
    case 10:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 11; setState('BACKING OFF', 'Leaving home'); }
      break;
    case 11:
      setMotor(-dir, sm.velocity);
      if (!sensorHOME) { stopMotor(); sm.phase = 12; log(`方法 ${method}：离开原点`); }
      break;
    case 12:
      if (Math.abs(velocity) < 0.5) { setMotor(dir, sm.lowVel); sm.phase = 13; setState('SEEKING HOME', 'Seeking Home Edge'); }
      break;
    case 13:
      setMotor(dir, sm.lowVel);
      // 核心区别：再次触碰原点瞬间立即停止
      if (sensorHOME) { stopMotor(); completeHoming('原点边缘位置 ' + position.toFixed(1)); }
      break;
  }
}

function tickEdgeType23_27(isForward) {
  const dir = isForward ? 1 : -1;
  const limitErr = isForward ? sensorNOT : sensorPOT;
  const limitRev = isForward ? sensorPOT : sensorNOT;
  const method = isForward ? 23 : 27;

  switch(sm.phase) {
    case 0:
      if (limitErr) { triggerError((isForward ? '负' : '正') + '限位有效，回零失败'); return; }
      if (sensorHOME) { setState('MOVING', 'Condition B'); setMotor(-dir, sm.velocity); sm.phase = 10; } 
      else { setState('SEEKING', 'Condition A'); setMotor(dir, sm.velocity); sm.phase = 1; }
      break;
    case 1:
      setMotor(dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { stopMotor(); sm.phase = 2; log(`方法 ${method}(A)：原点有效`); }
      else if (limitRev) { stopMotor(); sm.phase = 20; log(`方法 ${method}(C)：反向限位触发`); }
      break;
    case 2:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.lowVel); sm.phase = 3; setState('BACKING OFF', 'Leaving home'); }
      break;
    case 3:
      setMotor(-dir, sm.lowVel);
      if (limitRev) { triggerError('运动中限位触发'); return; }
      if (!sensorHOME) { stopMotor(); completeHoming('原点边缘位置 ' + position.toFixed(1)); }
      break;
    case 10:
      setMotor(-dir, sm.velocity);
      if (limitRev) { triggerError('运动中限位触发'); return; }
      if (!sensorHOME) { stopMotor(); sm.phase = 11; log(`方法 ${method}(B)：已离开原点`); }
      break;
    case 11:
      if (Math.abs(velocity) < 0.5) { setMotor(dir, sm.velocity); sm.phase = 1; setState('SEEKING', 'Executing Condition A'); }
      break;
    case 20:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 21; setState('RETURNING', 'Condition C'); }
      break;
    case 21:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { sm.phase = 22; }
      break;
    case 22:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (!sensorHOME) { stopMotor(); sm.phase = 23; log(`方法 ${method}(C)：已穿过原点`); }
      break;
    case 23:
      if (Math.abs(velocity) < 0.5) { setMotor(dir, sm.velocity); sm.phase = 1; setState('SEEKING', 'Executing Condition A'); }
      break;
  }
}

function tickEdgeType24_28(isForward) {
  const dir = isForward ? 1 : -1;
  const limitErr = isForward ? sensorNOT : sensorPOT;
  const limitRev = isForward ? sensorPOT : sensorNOT;
  const method = isForward ? 24 : 28;

  switch(sm.phase) {
    case 0:
      if (limitErr) { triggerError((isForward ? '负' : '正') + '限位有效，回零失败'); return; }
      if (sensorHOME) { setState('MOVING', 'Condition B'); setMotor(-dir, sm.velocity); sm.phase = 11; } 
      else { setState('SEEKING', 'Condition A'); setMotor(dir, sm.velocity); sm.phase = 1; }
      break;
    case 1:
      setMotor(dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { stopMotor(); sm.phase = 10; log(`方法 ${method}(A)：原点有效`); }
      else if (limitRev) { stopMotor(); sm.phase = 20; log(`方法 ${method}(C)：反向限位触发`); }
      break;
    case 10:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 11; setState('BACKING OFF', 'Leaving home'); }
      break;
    case 11:
      setMotor(-dir, sm.velocity);
      if (limitRev) { triggerError('运动中限位触发'); return; }
      if (!sensorHOME) { stopMotor(); sm.phase = 12; log(`方法 ${method}：离开原点`); }
      break;
    case 12:
      if (Math.abs(velocity) < 0.5) { setMotor(dir, sm.lowVel); sm.phase = 13; setState('SEEKING HOME', 'Seeking Home Edge'); }
      break;
    case 13:
      setMotor(dir, sm.lowVel);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { stopMotor(); completeHoming('原点边缘位置 ' + position.toFixed(1)); }
      break;
    case 20:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 21; setState('RETURNING', 'Condition C'); }
      break;
    case 21:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { sm.phase = 22; }
      break;
    case 22:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (!sensorHOME) { stopMotor(); sm.phase = 12; log(`方法 ${method}(C)：穿过原点，转执行找原点过程`); }
      break;
  }
}

function tickEdgeType25_29(isForward) {
  const dir = isForward ? 1 : -1;
  const limitErr = isForward ? sensorNOT : sensorPOT;
  const limitRev = isForward ? sensorPOT : sensorNOT;
  const method = isForward ? 25 : 29;

  switch(sm.phase) {
    case 0:
      if (limitErr) { triggerError((isForward ? '负' : '正') + '限位有效，回零失败'); return; }
      if (sensorHOME) { setState('MOVING', 'Condition B'); setMotor(dir, sm.velocity); sm.phase = 10; } 
      else { setState('SEEKING', 'Condition A'); setMotor(dir, sm.velocity); sm.phase = 1; }
      break;
    case 1:
      setMotor(dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { sm.phase = 2; log(`方法 ${method}(A)：进入原点`); }
      else if (limitRev) { stopMotor(); sm.phase = 20; log(`方法 ${method}(C)：反向限位触发`); }
      break;
    case 2:
      setMotor(dir, sm.velocity);
      if (limitRev) { stopMotor(); sm.phase = 20; }
      if (!sensorHOME) { stopMotor(); sm.phase = 3; log(`方法 ${method}：已离开原点`); }
      break;
    case 3:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.lowVel); sm.phase = 4; setState('SEEKING HOME', 'Seeking Home Edge'); }
      break;
    case 4:
      setMotor(-dir, sm.lowVel);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { stopMotor(); completeHoming('原点边缘位置 ' + position.toFixed(1)); }
      break;
    case 10:
      setMotor(dir, sm.velocity);
      if (limitRev) { stopMotor(); sm.phase = 20; }
      if (!sensorHOME) { stopMotor(); sm.phase = 3; log(`方法 ${method}(B)：已离开原点`); }
      break;
    case 20:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 21; setState('RETURNING', 'Condition C'); }
      break;
    case 21:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { stopMotor(); sm.phase = 22; log(`方法 ${method}(C)：原点有效停止`); }
      break;
    case 22:
      if (Math.abs(velocity) < 0.5) { sm.phase = 10; log(`方法 ${method}(C)：转执行B`); }
      break;
  }
}

function tickEdgeType26_30(isForward) {
  const dir = isForward ? 1 : -1;
  const limitErr = isForward ? sensorNOT : sensorPOT;
  const limitRev = isForward ? sensorPOT : sensorNOT;
  const method = isForward ? 26 : 30;

  switch(sm.phase) {
    case 0:
      if (limitErr) { triggerError((isForward ? '负' : '正') + '限位有效，回零失败'); return; }
      if (sensorHOME) { setState('MOVING', 'Condition B'); setMotor(dir, sm.velocity); sm.phase = 10; } 
      else { setState('SEEKING', 'Condition A'); setMotor(dir, sm.velocity); sm.phase = 1; }
      break;
    case 1:
      setMotor(dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { sm.phase = 2; log(`方法 ${method}(A)：进入原点`); }
      else if (limitRev) { stopMotor(); sm.phase = 20; log(`方法 ${method}(C)：反向限位触发`); }
      break;
    case 2:
    case 10:
      setMotor(dir, sm.velocity);
      if (limitRev) { stopMotor(); sm.phase = 20; }
      if (!sensorHOME) { stopMotor(); sm.phase = 3; log(`方法 ${method}：穿过原点，减速`); }
      break;
    case 3:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 4; setState('RETURNING', 'Returning to home'); }
      break;
    case 4:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { stopMotor(); sm.phase = 5; log(`方法 ${method}：原点再次有效`); }
      break;
    case 5:
      if (Math.abs(velocity) < 0.5) { setMotor(dir, sm.lowVel); sm.phase = 6; setState('BACKING OFF', 'Leaving home'); }
      break;
    case 6:
      setMotor(dir, sm.lowVel);
      if (limitRev) { triggerError('运动中限位触发'); return; }
      if (!sensorHOME) { stopMotor(); completeHoming('原点边缘位置 ' + position.toFixed(1)); }
      break;
    case 20:
      if (Math.abs(velocity) < 0.5) { setMotor(-dir, sm.velocity); sm.phase = 21; setState('RETURNING', 'Condition C'); }
      break;
    case 21:
      setMotor(-dir, sm.velocity);
      if (limitErr) { triggerError('运动中错误限位触发'); return; }
      if (sensorHOME) { stopMotor(); sm.phase = 5; log(`方法 ${method}(C)：原点有效停止`); }
      break;
  }
}
// ============================================================
// 方法 19-30：原点开关边缘检测 状态机路由层
// ============================================================

function tickMethod19(dt) { tickEdgeType19_21(true); }
function tickMethod20(dt) { tickEdgeType20_22(true); }
function tickMethod21(dt) { tickEdgeType19_21(false); }
function tickMethod22(dt) { tickEdgeType20_22(false); }

function tickMethod23(dt) { tickEdgeType23_27(true); }
function tickMethod24(dt) { tickEdgeType24_28(true); }
function tickMethod25(dt) { tickEdgeType25_29(true); }
function tickMethod26(dt) { tickEdgeType26_30(true); }

function tickMethod27(dt) { tickEdgeType23_27(false); }
function tickMethod28(dt) { tickEdgeType24_28(false); }
function tickMethod29(dt) { tickEdgeType25_29(false); }
function tickMethod30(dt) { tickEdgeType26_30(false); }

// ============================================================
// 方法33：负向搜索Z相
// ============================================================
function tickMethod33(dt) {
  setState('INDEX SEARCH', 'Direct Z search');
  setMotor(-1, sm.velocity);
  if (sensorNOT || sensorHOME || sensorPOT) {
    triggerError('运动中信号触发（限位/原点）');
    return;
  }
  if (getRisingEdge(sensorINDEX, prevSensorINDEX)) {
    log('方法 33：Index脉冲发现于 ' + position.toFixed(1));
    completeHoming('Z相位置 ' + position.toFixed(1));
  }
}

// ============================================================
// 方法34：正向搜索Z相
// ============================================================
function tickMethod34(dt) {
  setState('INDEX SEARCH', 'Direct Z search');
  setMotor(1, sm.velocity);
  if (sensorNOT || sensorHOME || sensorPOT) {
    triggerError('运动中信号触发（限位/原点）');
    return;
  }
  if (getRisingEdge(sensorINDEX, prevSensorINDEX)) {
    log('方法 34：Index脉冲发现于 ' + position.toFixed(1));
    completeHoming('Z相位置 ' + position.toFixed(1));
  }
}

// ============================================================
// 方法35/37：当前位置即原点
// ============================================================
function tickMethod35(dt) {
  completeHoming('当前位置 ' + position.toFixed(1) + ' 即为原点');
}
