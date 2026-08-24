"""Bridge between sparky-sim's strategy layer and real robot code under maple-sim.

This package sits *outside* `common_sim` on purpose. The import direction is:

    bridge/      --> imports from --> common_sim/, game_specific/
    common_sim/  --> never imports from --> bridge/

which is the same constraint `test/test_import_contract.py` enforces for
`game_specific`. The bridge is an adapter onto the strategy layer's duck-typed
contract; nothing in the simulator may grow a dependency on a running JVM.

Two channels, matching the feasibility brief:

* `operator`    -- Python -> robot. DriverStation state and joystick input over
                   the HALSim WebSocket server (port 3300). This is the same
                   path a physical Xbox controller takes, so the robot's binding
                   layer and its interlocks are exercised, not bypassed.
* `robot_state` -- robot -> Python. NetworkTables 4, reading what AdvantageKit
                   already publishes. No robot-code changes required.

`sim_process` launches and reaps the JVM that runs both.
"""
