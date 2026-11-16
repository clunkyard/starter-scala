// A robot on the arena's shared Java core binding.
// Bearing is earned: a scan contact's bearing is off by up to its
// bearingError (degrees, a fixed share of the arc), so a wide beam finds a
// robot and a narrow one aims at it. The robot narrows the beam, holds it
// on the target and aims along a line fitted through its recent sightings.
// Contacts within 120 units are exact.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
//   from_mount(heading pitch) mount_toward(point)
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   history(slot) last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) to_chassis(heading pitch attitude)
//   from_chassis(heading pitch attitude) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) debug.log(line) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
import clunkyard._
import scala.collection.JavaConverters._

object Bot {
  val SearchArc = 40.0 // degrees: sweeps the circle in half a second
  val HoldWidth = 60.0 // units across at the target: it stays in the beam from tick to tick
  val MinArc = 6.0 // degrees: the narrowest the beam holds
  val Window = 12 // ticks of sightings the aim averages
  val Sightings = 8 // narrow sightings in the window before a shot
  val LookEvery = 40 // ticks between looks around the target, for a nearer robot
  val Reacquire = 10 // ticks to look again where a lost target was

  /** A line through recent sightings: where the robot is now, how fast it moves, and how many sightings it rests on. */
  case class Fit(position: Vec3, velocity: Vec3, count: Int)

  def main(args: Array[String]): Unit = Clunkyard.run(new Bot)
}

class Bot extends Robot {
  import Bot._
  private var dodges = 0
  private var dodgeEnd = 0
  private var slot = 0 // the last target's slot, for looking again where it was lost
  private def steer(towards: Double, current: Double): Double = angles().clamp(angles().angleDiff(towards, current) / 45)

  // A straight line through the narrowest of the last Window ticks' sightings: where the
  // robot is now and how fast it moves. Each sighting is off by its bearingError; the line averages that out.
  private def fit(slot: Int, tick: Int): Option[Fit] = {
    val recent = tracker().history(slot).asScala.filter(_.tick() > tick - Window)
    if (recent.size < 2) return None
    val best = recent.map(_.bearingError()).min
    val seen = recent.filter(_.bearingError() <= 1.5 * best) // the narrow ones
    val n = seen.size
    if (n < 2) return None
    val t0 = seen.map(_.tick().toDouble).sum / n
    val spread = seen.map(s => (s.tick() - t0) * (s.tick() - t0)).sum
    // One axis: the line's position now and its slope per tick.
    def axis(of: Vec3 => Double): (Double, Double) = {
      val mean = seen.map(s => of(s.position())).sum / n
      val slope = seen.map(s => (s.tick() - t0) * (of(s.position()) - mean)).sum / spread
      (mean + slope * (tick - t0), slope * this.`match`().tickRate())
    }
    val (x, y, z) = (axis(_.x()), axis(_.y()), axis(_.z()))
    Some(Fit(new Vec3(x._1, y._1, z._1), new Vec3(x._2, y._2, z._2), n))
  }

  override def onTurn(): Unit = {
    val self = me()
    val position = self.position()
    val gun = this.gun()
    val scanner = this.scanner()
    val tick = clock().tick()
    drive().forces(Drive.brake(0.2))
    val target = sensors().nearest().orElse(null)
    val cue = tracker().cue().orElse(null)
    val lastSeen = if (slot != 0) tracker().lastSeen(slot) else java.util.OptionalInt.empty()
    val free = this.`match`().is3d() && arena().gravity().z() == 0
    if (target != null) {
      slot = target.slot()
      val throttle = if (target.range() > 350) 0.7 else if (target.range() < 200) -0.4 else 0.2
      drive().set(throttle, steer(target.bearing(), self.heading()), steer(target.elevation(), self.pitch()))
      // Halve the beam each tick until it is HoldWidth across at the target:
      // narrow enough for a precise bearing, wide enough to keep it in.
      val hold = math.max(MinArc, math.toDegrees(2 * math.atan2(HoldWidth / 2, target.range())))
      // In a crowd a narrow beam sees one robot. Every LookEvery ticks,
      // widen it for a tick: a nearer robot it finds becomes the target.
      val arc = if (tick % LookEvery == 0) SearchArc else math.max(hold, math.min(SearchArc, scanner.last().arc() / 2))
      scanner.set(target.bearing(), arc, target.elevation())
      val power = gun.powerForFlightTime(target.range(), 1)
      // An exact contact: gun.lead is the intercept if the target holds course.
      // A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
      val line = if (target.bearingError() > 0) fit(target.slot(), tick) else None
      val ready = target.bearingError() == 0 || line.exists(_.count >= Sightings)
      val aim =
        if (target.bearingError() > 0) line.map(f => gun.leadPoint(f.position, f.velocity, power).orElse(null)).orNull
        else gun.lead(target, power).orElse(null)
      val heading = if (aim == null) target.bearing() else aim.heading()
      val pitch = if (aim == null) target.elevation() else aim.pitch()
      gun.aim(heading, pitch)
      val now = gun.aimNow()
      if (ready && math.abs(angles().angleDiff(heading, now.heading())) < 3 && math.abs(pitch - now.pitch()) < 3 && gun.canFire(power, 10)) gun.fire(power)
      status().set(if (ready) "TRACK" else "NARROW", target.range())
    } else if (lastSeen.isPresent && tick - lastSeen.getAsInt <= Reacquire) {
      // Lost it: look again where it was last seen, widening the beam, before searching.
      val where = tracker().position(slot).orElse(position)
      val bearing = angles().bearingTo(position, where)
      val elevation = if (free) angles().elevationTo(position, where) else 0.0
      drive().set(0.5, steer(bearing, self.heading()), steer(elevation, self.pitch()))
      scanner.set(bearing, math.min(SearchArc, scanner.last().arc() * 2), elevation)
      status().set("REACQUIRE")
    } else if (cue != null) {
      val bearing = angles().bearingTo(position, cue.position())
      val distance = angles().distance(position, cue.position())
      val elevation = if (free) angles().elevationTo(position, cue.position()) else 0.0
      val error = math.min(cue.bearingError(), 30)
      val across = (tick % 5 - 2) / 2.0 * error
      val up = if (free) ((tick / 5) % 3 - 1) * error / 2 else 0.0
      drive().set(0.7, steer(bearing, self.heading()), steer(elevation, self.pitch()))
      scanner.set(bearing + across, scanner.arcForReach(distance + 100), elevation + up)
      status().set("CUE", distance)
    } else {
      drive().set(0.5, 0.33, steer(0, self.pitch()))
      val pitch = if (free) ((tick / 10) % 3 - 1) * 35.0 else 0.0
      scanner.set(angles().normalize(scanner.last().heading() + 36), SearchArc, pitch)
      status().set("SEARCH")
    }
    // A simple dodge: vary it, or predict the shooter.
    if (tick >= dodgeEnd && sensors().cues().asScala.exists(_.source() == Source.MUZZLE)) {
      dodges += 1
      dodgeEnd = tick + 10
    }
    if (tick < dodgeEnd) {
      // Off the line of fire: left, then right next time (turn is positive left, strafe positive right).
      val left = dodges % 2 == 1
      if (this.`match`().control() == Control.FORCES) {
        drive().set(Drive.throttle(0.5))
        drive().forces(Drive.strafe(if (left) -1 else 1))
      } else drive().set(1, if (left) 1 else -1)
      status().set(if (left) "dodge:left" else "dodge:right", dodges)
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
