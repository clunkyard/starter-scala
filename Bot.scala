// A robot on the arena's shared Java core binding.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range)
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
import clunkyard._

class Bot extends Robot {
  private def steer(towards: Double, current: Double): Double = angles().clamp(angles().angleDiff(towards, current) / 45)
  override def onTurn(): Unit = {
    val self = me()
    val position = self.position()
    val gun = this.gun()
    val scanner = this.scanner()
    drive().forces(Drive.brake(0.2))
    val target = sensors().nearest().orElse(null)
    val cue = tracker().cue().orElse(null)
    val free = this.`match`().is3d() && arena().gravity().z() == 0
    if (target != null) {
      val throttle = if (target.range() > 350) 0.7 else if (target.range() < 200) -0.4 else 0.2
      drive().set(throttle, steer(target.bearing(), self.heading()), steer(target.elevation(), self.pitch()))
      scanner.set(target.bearing(), 20, target.elevation())
      val power = gun.powerForFlightTime(target.range(), 1)
      val aim = gun.lead(target, power).orElse(null)
      val heading = if (aim == null) target.bearing() else aim.heading()
      val pitch = if (aim == null) target.elevation() else aim.pitch()
      gun.aim(heading, pitch)
      val now = gun.aimNow()
      if (math.abs(angles().angleDiff(heading, now.heading())) < 3 && math.abs(pitch - now.pitch()) < 3 && gun.canFire(power, 10)) gun.fire(power)
      status().set("TRACK", target.range())
    } else if (cue != null) {
      val bearing = angles().bearingTo(position, cue.position())
      val distance = angles().distance(position, cue.position())
      val elevation = if (free) angles().elevationTo(position, cue.position()) else 0.0
      val error = math.min(cue.bearingError(), 30)
      val tick = clock().tick()
      val across = (tick % 5 - 2) / 2.0 * error
      val up = if (free) ((tick / 5) % 3 - 1) * error / 2 else 0.0
      drive().set(0.7, steer(bearing, self.heading()), steer(elevation, self.pitch()))
      scanner.set(bearing + across, scanner.arcForReach(distance + 100), elevation + up)
      status().set("CUE", distance)
    } else {
      drive().set(0.5, 0.33, steer(0, self.pitch()))
      val pitch = if (free) ((clock().tick() / 10) % 3 - 1) * 35.0 else 0.0
      scanner.set(angles().normalize(scanner.last().heading() + 36), 40, pitch)
      status().set("SEARCH")
    }
    if (battery().charge() < battery().capacity() / 4) drive().set(Drive.throttle(drive().get("throttle").orElse(0) * 0.5))
    val storm = nav().storm().orElse(null)
    if (storm != null && storm.margin(position).orElse(-1e9) > -60) {
      val away = angles().bearingTo(storm.center(), position)
      drive().set(Drive.throttle(0.8).turn(angles().clamp(angles().angleDiff(away, self.heading()) / 30)))
    }
    val zone = nav().zone().orElse(null)
    if (zone != null && zone.margin(position).orElse(1e9) < 60) {
      val climb = if (zone.shape() == Shape.SPHERE) angles().elevationTo(position, zone.center()) else 0.0
      drive().set(0.8, angles().clamp(angles().angleDiff(angles().bearingTo(position, zone.center()), self.heading()) / 30), angles().clamp((climb - self.pitch()) / 30))
    }
  }
}

object Bot {
  def main(args: Array[String]): Unit = Clunkyard.run(new Bot)
}
