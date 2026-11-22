# Direct wheelstand request from the dock

Unit 3 was on the charging platform when the SDK submitted PopAWheelie with connected cube ID 1 and zero retries. Cube connection succeeded and accelerometer packets were arriving. No robot_observed_object callback was recorded during this session. The engine acknowledged the request and returned BAD_OBJECT 0.482 s after submission. Both wheel speeds stayed zero across all 16 states in the following second. This was a rejected command, with no physical wheelstand or cube contact. Handback was on the dock at 4.075 V with battery level full and the quiet switch on.

[Two corrected shelf views](dock-request.mp4) show the command at approximately 2.6 s. [SDK telemetry and action responses](sdk.jsonl.gz) and [58 raw accelerometer packets](cube.csv) include connection and handback.
