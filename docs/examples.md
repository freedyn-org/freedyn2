# Examples

Every example comes with FreeDyn: `freedyn2 examples` lists them, `freedyn2 examples NAME`
copies one into the current directory, `freedyn2 view NAME.yaml` opens it.

The viewers on this page are the real viewer, exported with `freedyn2 view --export`: drag to
rotate, space to play, click a channel to plot it. They run in your browser without FreeDyn
(tuning and the other server functions need `freedyn2 view`).

## Quarter car over a bump

The model of the [tutorial](tutorial.md): a quarter of a car body on its suspension, a wheel
on a tire spring, a cosine bump at 10 m/s. Parameters for every spring and damper.

<iframe src="LIVE/quarter_car.html" width="100%" height="620" loading="lazy" style="border:1px solid #8884;border-radius:6px"></iframe>

`freedyn2 examples quarter_car`

## A car on an OpenCRG road

The car of the [OpenCRG tutorial](tutorial-crg.md): built from hardpoints, built-in tires on an
OpenCRG test track with a speed bump, washboard, a pothole under the left wheels and banking;
a speed controller and a sensor at the driver's head.

<iframe src="LIVE/car_on_crg.html" width="100%" height="620" loading="lazy" style="border:1px solid #8884;border-radius:6px"></iframe>

`freedyn2 examples car_on_crg`

## Double-wishbone suspension

A front suspension on a rig, built from [hardpoints](reference/hardpoints.md): control arms,
wheel carrier, tie rod and wheel, a coil-over on the lower arm, a tire spring on a road with a
bump. Tick *hardpoints* under View to see the named points; a sensor at the wheel center
gives the vertical acceleration from the solver.

<iframe src="LIVE/double_wishbone.html" width="100%" height="620" loading="lazy" style="border:1px solid #8884;border-radius:6px"></iframe>

`freedyn2 examples double_wishbone`

## Double pendulum

Two rods on hinges, released from the horizontal – chaotic: tiny changes of the start make a
different motion after a few seconds (try it: change a mass in the file with the viewer open).

<iframe src="LIVE/double_pendulum.html" width="100%" height="620" loading="lazy" style="border:1px solid #8884;border-radius:6px"></iframe>

`freedyn2 examples double_pendulum`

## Dzhanibekov effect

A free body spinning about its middle principal axis flips over and over – the intermediate
axis theorem. A test of the integrator: energy and angular momentum stay constant.

<iframe src="LIVE/dzhanibekov.html" width="100%" height="620" loading="lazy" style="border:1px solid #8884;border-radius:6px"></iframe>

`freedyn2 examples dzhanibekov`

## Elastic pendulum

A body hung on a spring at a point off its center of mass, started sideways, without damping:
a 3D motion that exchanges energy between swinging, bouncing and turning.

<iframe src="LIVE/spring_pendulum.html" width="100%" height="620" loading="lazy" style="border:1px solid #8884;border-radius:6px"></iframe>

`freedyn2 examples spring_pendulum`

## Driven rotor

A rotor driven at a constant speed (a driver on an angle), and a flywheel coupled to it by a
torsion spring: the flywheel oscillates about the rotor's motion.

<iframe src="LIVE/driven_rotor.html" width="100%" height="620" loading="lazy" style="border:1px solid #8884;border-radius:6px"></iframe>

`freedyn2 examples driven_rotor`

## More

`freedyn2 examples` lists the others: oscillators, a torsion pendulum, a bushing mount, a
spline spring, two free bodies under internal forces, a winding spring, an oscillator and a
driven cart moved by runtime inputs (`input_oscillator`). In the repository:
`examples/scaling/` (the legacy comparison car), `examples/compare/` (legacy `.fds` models).
