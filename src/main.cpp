#include "wrapper.h"
#include <QApplication>

int main (int argc, char *argv[]) {
    QApplication app(argc, argv);
	
    Wrapper wrapper;
    // wrapper.loadCloud(); // Debugging
	wrapper.show ();

	return app.exec ();
}
