# Team working agreements (read by humans and AI assistants)

## Where code lives
- All team code goes in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
- `FtcRobotController/` is FIRST's SDK plus sample OpModes. **Never edit it.**
  Copy a sample into `teamcode/` first, then modify the copy.

## Pedro Pathing (our chassis stack)
- The drivetrain + localizer + algorithm are assembled in
  `teamcode/pedro/Constants.java` (`Constants.create()`).
- TeleOp drives through `follower.manual(new DrivePowers(...))` and calls
  `follower.update()` every loop iteration so localization stays alive.
- Order of work: fill in `Constants` → run the tuner OpModes until they pass
  → TeleOp → Autonomous. Never skip tuning.

## FTC pitfalls to always check
- Every `hardwareMap.get()` name must exactly match the robot configuration
  on the Robot Controller (a typo = crash at INIT).
- Fetch ALL hardware during init, never inside the loop.
- Keep telemetry light inside loops; heavy telemetry slows driving.
- `gamepad1` = driver (chassis), `gamepad2` = operator (mechanisms).

## Build and deploy
- Build: `./gradlew assembleDebug` from the project root.
- Deploy: install the APK from `TeamCode/build/outputs/apk/debug/` via
  Android Studio Run or `adb install -r`.
- In OpenCode, `/build` and `/deploy` do the above.

## Git habits
- `main` is always deployable. Features go in `name/what` branches.
- Commit early and often; push at the end of every work session.
- Robot config exports live in `TeamCode/configs/` so a lost config never
  kills a competition day.
