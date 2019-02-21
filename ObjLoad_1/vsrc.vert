#version 330
uniform mat4 uPMatrix,uVMatrix,uMMatrix;
layout (location = 0) in vec3 aPosition;

void main(void)
{
    gl_Position = uPMatrix * uVMatrix * uMMatrix * vec4(aPosition,1);
  // gl_Position = vec4(aPosition.x, aPosition.y, aPosition.z, 1.0);
}
