#include "openglviewer.h"

OpenGLViewer::OpenGLViewer(QWidget* parent) :
    QOpenGLWidget(parent),
    m_isFirstMouseIn(true),
    m_mouseCurrentX(0),
    m_mouseCurrentY(0),
    m_cloud(nullptr),
    m_needInitialize(false),
    m_animate(false),
    m_stepLRrotate(0),
    m_stepUDrotate(0),
    m_stepFBmove(0),
    m_shader(nullptr),
    m_shaderAttrColor(0),
    m_shaderAttrPosition(0),
    m_shaderUnifromTransform(0) {

}

OpenGLViewer::~OpenGLViewer() {

}

void OpenGLViewer::initializeGL() {
    initializeOpenGLFunctions();
    initializeShader();
    initializeMovementEvent();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

void OpenGLViewer::render() {
    if(m_needInitialize == true) {
        initializeGL();
        m_needInitialize = false;
    }

    resizeGL(width(), height());
    renderGL();

	emit postUpdateEvent();
}

void OpenGLViewer::renderGL() {
    makeCurrent();

    // Background
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	if (m_cloud != nullptr) {
		// Active shader
		m_shader->bind();
		setShaderParameters();

		// Draw it
		glVertexAttribPointer(m_shaderAttrPosition, sizeof(Position) / sizeof(float), GL_FLOAT, GL_FALSE, sizeof(CloudPoint), (GLvoid*)(&(m_cloud->points->position)));
		glVertexAttribPointer(m_shaderAttrColor, sizeof(Color) / sizeof(float), GL_FLOAT, GL_FALSE, sizeof(CloudPoint), (GLvoid*)(&(m_cloud->points->color)));

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);

		glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_cloud->size));

		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(0);

		m_shader->release();
	}
}

void OpenGLViewer::resizeGL(int width, int height) {
    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, width * retinaScale, height * retinaScale);
    doMovementEvent();
}

void OpenGLViewer::paintGL() {

}

inline void OpenGLViewer::initializeShader() {
    const char* pathToVertexShader = "../src/VertexShader.glsl";
    const char* pathToFragmentShader = "../src/FragmentShader.glsl";

    std::ifstream vertexShaderFStream, fragmentShaderFStream;
    std::string vertexShaderSourceCode, fragmentShaderSourceCode;

    vertexShaderFStream.exceptions(std::ifstream::badbit);
    fragmentShaderFStream.exceptions(std::ifstream::badbit);

    try {
        vertexShaderFStream.open(pathToVertexShader, std::ios::in);
        fragmentShaderFStream.open(pathToFragmentShader, std::ios::in);
        std::stringstream vertexSStream, fragmentSStream;

        vertexSStream << vertexShaderFStream.rdbuf();
        fragmentSStream << fragmentShaderFStream.rdbuf();

        vertexShaderSourceCode = vertexSStream.str();
        fragmentShaderSourceCode = fragmentSStream.str();
    } catch(std::ifstream::failure e) {
        std::cout << "FAILURE: VERTEX SHADER FILE OR FRAGMENT SHADER FILE DOES NOT EXIST." << std::endl;
        exit(-1);
    }

    m_shader = new QOpenGLShaderProgram(this);
    m_shader->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSourceCode.c_str());
    m_shader->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSourceCode.c_str());
    m_shader->link();

   // Locate Attr Variable
    m_shaderAttrPosition = m_shader->attributeLocation("positionAttr");
    m_shaderAttrColor = m_shader->attributeLocation("colorAttr");
    // Locate Uniform Variable
    m_shaderUnifromTransform = m_shader->uniformLocation("transform");
}

inline void OpenGLViewer::setShaderParameters() {
    /* Process coordination transformation */
    QMatrix4x4 transform;
    transform.perspective(60.0f, 4.0f/3.0f, 0.1f, 100.0f);
    transform.translate(0, 0, -0.0f + m_stepFBmove);
    transform.rotate(0.0f + m_stepLRrotate, QVector3D(0.0f, 1.0f, 0.0f));
    transform.rotate(0.0f + m_stepUDrotate,QVector3D(1.0f, 0.0f, 0.0f));
    m_shader->setUniformValue(m_shaderUnifromTransform, transform);
}

void OpenGLViewer::initializeMovementEvent() {
    for(size_t i = 0; i < 256; i ++) {
        m_keyStatus[i] = OPENGL_KEY_RELEASED;
    }
}

void OpenGLViewer::setCloud(Cloud *cloud) {
    m_cloud = cloud;
}

void OpenGLViewer::setAnimate(bool animate, Cloud* cloud) {
	if(cloud != nullptr) setCloud(cloud);
    m_animate = animate;
    if(m_animate == true) {
        render();
    }
}

void OpenGLViewer::doMovementEvent() {
    doKeyMovementEvent();
    doMouseMovementEvent();
}

void OpenGLViewer::doKeyMovementEvent() {
    for (size_t i = 0; i < 256; i++) {
        if (m_keyStatus[i] == OPENGL_KEY_PRESSED) {
            switch (i) {
            case Qt::Key_Equal: m_stepFBmove += 2.0f; break;
            case Qt::Key_Minus: m_stepFBmove -= 2.0f; break;
            case Qt::Key_W: m_stepUDrotate -= 2.0f; break;
            case Qt::Key_S: m_stepUDrotate += 2.0f; break;
            case Qt::Key_A: m_stepLRrotate -= 2.0f; break;
            case Qt::Key_D: m_stepLRrotate += 2.0f; break;
            }
        }
    }
}

void OpenGLViewer::doMouseMovementEvent() {

}

void OpenGLViewer::keyPressEvent(QKeyEvent* event) {
    if(m_animate) {
#ifdef TEST_
        std::cout << "Key Pressed: " << event->key() << ": ";
        switch(event->key()) {
        case Qt::Key_W: std::cout << "W"; break;
        case Qt::Key_S: std::cout << "S"; break;
        case Qt::Key_A: std::cout << "A"; break;
        case Qt::Key_D: std::cout << "D"; break;
        case Qt::Key_AddFavorite: std::cout << "+"; break;
        case Qt::Key_Minus: std::cout << "-"; break;
        default: std::cout << "others";
        }
        std::cout << std::endl;
#endif

        if(event->key() < 0 || event->key() > 255) {
            return ;
        }

        m_keyStatus[event->key()] = OPENGL_KEY_PRESSED;
    }
}

void OpenGLViewer:: keyReleaseEvent(QKeyEvent* event) {
    if(m_animate) {
#ifdef TEST_
        std::cout << "Key Released: " << event->key() << ": ";
        switch(event->key()) {
        case Qt::Key_W: std::cout << "W"; break;
        case Qt::Key_S: std::cout << "S"; break;
        case Qt::Key_A: std::cout << "A"; break;
        case Qt::Key_D: std::cout << "D"; break;
        case Qt::Key_AddFavorite: std::cout << "+"; break;
        case Qt::Key_Minus: std::cout << "-"; break;
        default: std::cout << "others";
        }
        std::cout << std::endl;
#endif

        if(event->key() < 0 || event->key() > 255) {
            return ;
        }

        m_keyStatus[event->key()] = OPENGL_KEY_RELEASED;
    }
}

void OpenGLViewer::mouseMoveEvent(QMouseEvent* event) {
    if(m_animate) {
        if(event->buttons() & Qt::LeftButton) {
    #ifdef TEST_
        std::cout << "Mouse Left Button Pressed and Movement: (" << event->x() << ", " << event->y() << ")" <<  std::endl;
    #endif

            if (m_isFirstMouseIn) {
                m_mouseCurrentX = -(event->x());
                m_mouseCurrentY = -(event->y());
            }

            if ((fabs(m_mouseCurrentX - event->x()) < DBL_EPSILON) &&
                (fabs(m_mouseCurrentY -event->y()) < DBL_EPSILON)
                ) {
                // Position is not changed
                return;
            }

            m_mouseCurrentX = event->x();
            m_mouseCurrentY = event->y();
        }
    }
}
