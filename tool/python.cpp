#include <sofa/gui/qt/viewer/qgl/QtGLViewer.h>
#include <QApplication>
#include <QGLViewer/qglviewer.h>


#include <sofa/simulation/common/Simulation.h>

// TODO: preprocessor based on SOFA_GUI_QGLVIEWER

extern "C" {

  void revolve_around(double x, double y, double z);
  void set_animation_period(unsigned i);
  
}



namespace {
  
  struct viewer_type {

	QGLViewer* instance;
	

	viewer_type() : instance(0) {
	  
	}

	void find() {
	  foreach (QWidget *widget, QApplication::topLevelWidgets()) {
		if(!instance){
		  instance = widget->findChild<sofa::gui::qt::viewer::qgl::QtGLViewer*>("viewer");
		}
	  }
	}


	QGLViewer* operator()() {
	  if(!instance) find();
	  return instance;
	}

  };

  static viewer_type viewer;
}




void revolve_around(double x, double y, double z) {

  if (!viewer()) {
	std::cout << "revolve_around: cant find qglviewer lol" << std::endl;
	return;

  }

  qglviewer::Vec pos(x, y, z);
  
  viewer()->camera()->setRevolveAroundPoint( pos );
}


void set_animation_period(unsigned i) {

  if (!viewer()) {
	std::cout << "revolve_around: cant find qglviewer lol" << std::endl;
	return;
  }

  std::cout << viewer()->animationPeriod() << std::endl;
  viewer()->setAnimationPeriod( i );
}


extern "C" {

  sofa::simulation::Simulation*  simulation() {
	return sofa::simulation::getSimulation();
  }

  sofa::simulation::Node* simulation_root(sofa::simulation::Simulation* sim) {
	return sim->GetRoot().get();
  }

  sofa::simulation::Node* node_child(sofa::simulation::Node* node, const char* name) {
	return node->getChild( name );
  }

  void node_each_child(sofa::simulation::Node* node, void (*func)(sofa::simulation::Node*) ) {
	sofa::simulation::Node::Children children = node->getChildren();

	for(unsigned i = 0, n = children.size(); i < n; ++i) {
	  // TODO safer cast ?
	  func( (sofa::simulation::Node*) (children[i]) );
	}
	
  }

  sofa::core::objectmodel::BaseObject* node_object(sofa::simulation::Node* node, const char* name) {
	return node->getObject( name );
  }

  const char* node_name(sofa::simulation::Node* node) {
	return node->getName().c_str();
  }


  const char* object_name(sofa::core::objectmodel::BaseObject* obj) {
	return obj->getName().c_str();
  }


}
