# Thought Process

**Premise**
_Sorry for the quantity of comments, i know they can be overwelming, but its just my optimal way of work._

Also,
_i mainly programmed in C in the past and never c++, so sometimes i maybe tend to use conventions for syntax (like variable name, ecc...) used in C or other programming languages_.

## Synchronization & Concurrency Model

I made the concurrency model so that it prioritizes not having seg faults or corruption of datas, using mutex as requested but also a wrapper like `std::unique_lock` and `std::lock_guard` to prevent lock leaks. 

Every transmitted frame get copied also into the emitted section so the tester can read a pure "log" of all the datas that has been sent trough the CAN.

To all the `deque`s have been applied a FIFO logic (First In, First Out).

## Testing and CI workflow
All the test have been added at the end. All of them uses only the "public" output of the simulator, not the private in the ECUs.

### Completed test method (added)
- test_app_brake(context);
- test_track_ini_throttle(context);
- test_plaus_throttle(context);
- test_rtd_on_manual(context);
- test_crit_safe_state(context);

