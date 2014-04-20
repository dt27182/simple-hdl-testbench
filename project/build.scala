import sbt._
import Keys._
//val extracted: Extracted = Project.extract(state)
//import extracted._

object BuildSettings extends Build {
  lazy val chisel = Project("chisel", file("chisel"))
  lazy val fsm = Project("fsm", file("src/fsm")).dependsOn(chisel)
  lazy val cpu = Project("cpu", file("src/cpu")).dependsOn(chisel)
  lazy val plustwo = Project("plustwo", file("src/plustwo")).dependsOn(chisel)
}
