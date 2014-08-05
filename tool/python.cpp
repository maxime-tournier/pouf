#include <sofa/gui/qt/viewer/qgl/QtGLViewer.h>
#include <QApplication>
#include <QGLViewer/qglviewer.h>



extern "C" {

  void revolve_around(double x, double y, double z);

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

