"""Function for generating an S-curve profile."""
import math
import matplotlib.pyplot as plt

def generate_s_curve_profile(max_v, max_a, time_to_max_a, dt, goal):
    """Returns an s-curve profile with the given constraints.
    Returns:
    t_rec -- list of timestamps
    x_rec -- list of positions at each timestep
    v_rec -- list of velocities at each timestep
    a_rec -- list of accelerations at each timestep
    Keyword arguments:
    max_v -- maximum velocity of profile
    max_a -- maximum acceleration of profile
    time_to_max_a -- time from rest to maximum acceleration
    dt -- timestep
    goal -- final position when the profile is at rest
    """
    t_rec = [0.0]
    x_rec = [0.0]
    v_rec = [0.0]
    a_rec = [0.0]
    
    j = max_a / time_to_max_a
    short_profile = max_v * (time_to_max_a + max_v / max_a) > goal
    
    if short_profile:
        profile_max_v = max_a * (
            math.sqrt(goal / max_a - 0.75 * time_to_max_a**2) - 0.5 *
                time_to_max_a
        )
    else:
        profile_max_v = max_v
    
    # Find times at critical points
    t2 = profile_max_v / max_a
    t3 = t2 + time_to_max_a
    if short_profile:
        t4 = t3
    else:
        t4 = goal / profile_max_v
    t5 = t4 + time_to_max_a
    t6 = t4 + t2
    t7 = t6 + time_to_max_a
    time_total = t7

    print(f"1: {time_to_max_a}  2: {t2}  3: {t3}  4 (max vel): {t4}  5: {t5}  6: {t6}  7: {t7}")
    areaOfTrapezoid = (t3 + t2 - time_to_max_a) / 2 * max_a # area represents final max vel
    print(f"Area: {areaOfTrapezoid}  Dist Traveled: {areaOfTrapezoid / 2 * t3}")


    while t_rec[-1] < time_total:
        t = t_rec[-1] + dt
        t_rec.append(t)
        if t < time_to_max_a:
            # Ramp up acceleration
            a_rec.append(j * t)
            v_rec.append(0.5 * j * t**2)
        elif t < t2:
            # Increase speed at max acceleration
            a_rec.append(max_a)
            v_rec.append(max_a * (t - 0.5 * time_to_max_a))
        elif t < t3:
            # Ramp down acceleration
            a_rec.append(max_a - j * (t - t2))
            v_rec.append(max_a * (t - 0.5 * time_to_max_a) - 0.5 * j * (t -
            t2) ** 2)
        elif t < t4:
            # Maintain max velocity
            a_rec.append(0.0)
            v_rec.append(profile_max_v)
        elif t < t5:
            # Ramp down acceleration
            a_rec.append(-j * (t - t4))
            v_rec.append(profile_max_v - 0.5 * j * (t - t4) ** 2)
        elif t < t6:
            # Decrease speed at max acceleration
            a_rec.append(-max_a)
            v_rec.append(max_a * (t2 + t5 - t - 0.5 * time_to_max_a))
        elif t < t7:
            # Ramp up acceleration
            a_rec.append(-max_a + j * (t - t6))
            v_rec.append(
            max_a * (t2 + t5 - t - 0.5 * time_to_max_a) + 0.5 * j * (t -
                t6) ** 2
            )
        else:
            a_rec.append(0.0)
            v_rec.append(0.0)
        x_rec.append(x_rec[-1] + v_rec[-1] * dt)
    
    return t_rec, x_rec, v_rec, a_rec

t_vals, pos_vals, vel_vals, accel_vals = generate_s_curve_profile(80, 180, 0.25, 0.01, 50)

fig, [ax1, ax2] = plt.subplots(2, 1, sharex=True)
ax1.set(xlabel='time (s)', ylabel='vel',
       title='Vel vs. Time')
ax1.plot(t_vals, vel_vals)
ax1.plot(t_vals, pos_vals)

ax2.set(xlabel='time (s)', ylabel='accel',
       title='Accel vs. Time')
ax2.plot(t_vals, accel_vals)
plt.show()