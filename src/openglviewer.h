#ifndef __OPENGLVIEWER_H__
#define __OPENGLVIEWER_H__

#include "common.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QCoreApplication>
#include <QtGui/QMatrix4x4>
#include <QtGui/QOpenGLShaderProgram>
#include <QtGui/QOpenGLFunctions>

#define OVHEADER(x) \
	"[OPENGL VIEWER]: " + std::string(x)

class OpenGLViewer : public QOpenGLWidget, protected QOpenGLFunctions {

Q_OBJECT

signals:
	void postInfo(const std::string& info);
	void postError(const std::string& error);
	void postUpdateEvent();

public:
    enum { OPENGL_KEY_PRESSED, OPENGL_KEY_RELEASED };

public:
    OpenGLViewer(QWidget* parent = 0);
    ~OpenGLViewer();
    void setCloud(Cloud* cloud);

protected:
    void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
    void keyReleaseEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE; // bad behavior
    void initializeGL() Q_DECL_OVERRIDE;
    void resizeGL(int width, int height) Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;

private:
    inline void initializeShader();
    inline void setShaderParameters();
    inline void initializeMovementEvent();
    inline void doMovementEvent();
    void doKeyMovementEvent();
    void doMouseMovementEvent(); // bad behavior
    void render();

private:
	bool m_needInitialize;

    uint m_keyStatus[256];
    bool m_isFirstMouseIn;
    double m_mouseCurrentX, m_mouseCurrentY; // bad behavior

    Cloud* m_cloud;

    float m_stepLRrotate;
    float m_stepUDrotate;
    float m_stepFBmove;

    QOpenGLShaderProgram* m_shader;
    uint m_shaderAttrPosition;
    uint m_shaderAttrColor;
    uint m_shaderUnifromTransform;
};

#endif // __OPENGLVIEWER_H__
