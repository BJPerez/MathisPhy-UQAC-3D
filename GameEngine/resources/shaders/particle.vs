#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aCenter;
layout (location = 2) in vec3 aColor;
out vec3 circleCenter;
out vec3 color;
void main()
{
   float x = 2.0*(aPos.x/800.0)-1;
   float y = 2.0*(aPos.y/600.0)-1;
   float z = aPos.z;
   gl_Position = vec4(x, y, z, 1.0);
   circleCenter = aCenter;
   color = aColor;
}