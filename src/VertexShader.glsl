attribute highp vec4 positionAttr;
attribute lowp vec4 colorAttr;

varying lowp vec4 color;
uniform highp mat4 transform;

void main() {
   color = colorAttr;
   gl_Position = transform * positionAttr;
}