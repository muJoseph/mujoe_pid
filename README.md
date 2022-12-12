# mujoe_pid

C implementation of the parallel PID controller seen below.

![image](https://user-images.githubusercontent.com/5027131/206946790-bd493dfe-8533-4656-8a61-e66577dcd059.png)

# Features:

• Derivative low-pass-filter (LPF) with configurable pole location

• Configurable controller output saturation

• Configurable anti-windup functionality


# Quickstart:

Create a global PID context structure.

```c

mujoe_pid_t mujoe_pid;

```

Create and initialize a PID context configuration structure.

```c

// Initialize PID context configuration structure with demo parameters:
// Kp = 1
// Ki = 2
// Kd = 0.0125
// Sample time: 100 ms
// LPF 3dB frequency: 20 Hz
// Output saturation: disabled
// Anti-windup: disabled
mujoe_pid_cfg_t pid_cfg;
mujoe_pid_initConfigParams( MUJOE_PID_CFG_PRESET_ID_DEMO, &pid_cfg );

```

Initialize the PID context.

```c

// Initialize PID context
mujoe_pid_initCtx( &mujoe_pid, &pid_cfg);

```

    
Call mujoe_pid_run() every T seconds.

```c

float dummy_setpoint = 1;
float dummy_plant_output = 0;
float dummy_err = 0;
float control = 0;
	
// Terminal loop
for(;;)
{
  // Compute closed loop error
  dummy_err = dummy_setpoint - plant_output;
		
  // Run PID, call every T seconds (100 ms)
  control = mujoe_pid_run( &mujoe_pid, dummy_err );

  // Drive dummy plant
  plant_output = dummy_plant( control );
}
  
```

# More Info:

Refer to doc/mujoe_pid_doc.pdf for theory and validation.

Refer to test/ folder for MATLAB  files used for validation.

Refer to example/main.c for example usage.

