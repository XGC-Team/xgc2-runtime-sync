# Acceptance Checklist

## P0

- [ ] realtime_sample 无 ACK。
- [ ] Late sample 不进入 Fresh。
- [ ] WrongCycle sample 不进入 Fresh。
- [ ] realtime queue age <= 1 cycle。
- [ ] CycleSnapshot 包含每 peer/channel 状态。
- [ ] 起飞前 session start 拒绝非地面站 chrony source。
- [ ] 起飞前 offset/uncertainty 超过 2 ms 时拒绝 start。
- [ ] leap status 非 Normal 时拒绝 start。

## P1

- [ ] LinkHealth 按 peer/channel 输出。
- [ ] WeaknetStateMachine 可迁移且有 reason/evidence。
- [ ] Ground Station 可显示 per-edge 指标。
- [ ] RuntimeHealth/XRPC status 输出 offset、uncertainty、quality、source、phase。
- [ ] 飞行中 clock 异常只标记 degraded/bad，不执行 step。
- [ ] VRPN 10-30 ms age 诊断要求时钟误差先压到 1-2 ms 量级。

## P2

- [ ] TrafficBudget 启动前报告。
- [ ] PayloadGuard 保护 oversized realtime payload。
- [ ] SendStagger offset 可复现，可记录。

## P3

- [ ] Recommendation 只输出建议，不主动降级。
- [ ] Query Side Path 不阻塞实时周期。
- [ ] Topology Profile 可切换。

## P4

- [ ] ROS1 package 可编译。
- [ ] launch 可启动 runtime 和 ground referee。
- [ ] netem 矩阵生成验收报告。
- [ ] check_chrony.sh 可用 fake chronyc 验证 PASS/FAIL，且不调用 makestep。
- [ ] 文档明确同步动作使用未来 T_exec，不在动作前临时重新校时。
