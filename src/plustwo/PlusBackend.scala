package plustwo
import Chisel._
import scala.collection.mutable.ArrayBuffer
import scala.collection.mutable.HashMap
import scala.collection.mutable.HashSet
import scala.collection.mutable.{Queue=>ScalaQueue}

trait PlusTwoTransform extends Chisel.Backend {
  private def addPlusOne(top: Module) :  Unit = {
    //instantiate plusone by setting up the compStack and using Module.apply
    Module.compStack.clear()
    Module.compStack.push(top)
    val plusone = Module(new PlusOne)
    plusone.genAllMuxes
    //connect plusone module inputs and outputs
    plusone.io.in.inputs += top.asInstanceOf[plustwo.DUT].io.out.inputs(0)
    top.asInstanceOf[plustwo.DUT].io.out.inputs(0) = plusone.io.out
  }

  preElaborateTransforms += ((top: Module) => levelChildren(top))
  preElaborateTransforms += ((top: Module) => {Module.sortedComps = gatherChildren(top).sortWith((x,y) => (x.level < y.level || (x.level == y.level && x.traversal < y.traversal)) )})
  preElaborateTransforms += ((top: Module) => collectNodesIntoComp(initializeDFS))
  preElaborateTransforms += ((top: Module) => addPlusOne(top))
}

class PlusCppBackend extends Chisel.CppBackend with PlusTwoTransform
