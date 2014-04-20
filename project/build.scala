import sbt._
import Keys._
//val extracted: Extracted = Project.extract(state)
//import extracted._

object BuildSettings extends Build {
  lazy val chisel = Project("chisel", file("chisel"))
  lazy val common = Project("common", file("src/common")).dependsOn(chisel)
  lazy val fsm = Project("fsm", file("src/fsm")).dependsOn(chisel, common)
  lazy val cpu = Project("cpu", file("src/cpu")).dependsOn(chisel, common)
  lazy val plustwo = Project("plustwo", file("src/plustwo")).dependsOn(chisel)
}
