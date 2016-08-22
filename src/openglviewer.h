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

//QT_BEGIN_NAMESPACE
//class QPainter;
//class QOpenGLContext;
//class QOpenGLPaintDevice;
//QT_END_NAMESPACE

class OpenGLViewer : public QOpenGLWidget, public QOpenGLFunctions {

Q_OBJECT

signals:
	void postUpdateEvent();

public:
    enum { OPENGL_KEY_PRESSED, OPENGL_KEY_RELEASED };

public:
    OpenGLViewer(QWidget* parent = 0);
    ~OpenGLViewer();
    void setCloud(Cloud* cloud);
    void setAnimate(bool animate, Cloud* cloud = nullptr);

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
    void renderGL();

private:
    uint m_keyStatus[256];
    bool m_isFirstMouseIn;
    double m_mouseCurrentX, m_mouseCurrentY; // bad behavior

    bool m_needInitialize;
    bool m_animate;
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
