# filters

The starter code is written in Python and depends on NumPy and Matplotlib.
This README gives a brief overview of each file.

- `localization.py` -- This is your main entry point for running experiments.
- `soccer_field.py` -- This implements the dynamics and observation functions, as well as the noise models for both. Add your Jacobian implementations here!
- `utils.py` -- This contains assorted plotting functions, as well as a useful
  function for normalizing angles.
- `policies.py` -- This contains a simple policy, which you can safely ignore.
- `ekf.py` -- Add your extended Kalman filter implementation here!
- `pf.py` -- Add your particle filter implementation here!

## Command-Line Interface

To visualize the robot in the soccer field environment, run
```bash
$ python localization.py --plot none
```
The blue line traces out the robot's position, which is a result of noisy actions.
The green line traces the robot's position assuming that actions weren't noisy.

After you implement a filter, the filter's estimate of the robot's position will be drawn in red.
```bash
$ python localization.py --plot ekf
$ python localization.py --plot pf
```

You can scale the noise factors for the data generation process or the filters
with the `--data-factor` and `--filter-factor` flags. To see other command-line
flags available to you, run
```bash
$ python localization.py -h
```
