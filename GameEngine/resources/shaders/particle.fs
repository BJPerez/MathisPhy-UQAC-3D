#version 330 core
out vec4 FragColor;
in vec3 circleCenter;
in vec3 color;
uniform float circleRadius;
void main()
{
   vec2 fragCoord = gl_FragCoord.xy;
   float squareDistFromCenter = pow(fragCoord.x - circleCenter.x, 2) + pow(fragCoord.y - circleCenter.y, 2);
   if (squareDistFromCenter > pow(circleRadius,2))
   {
       discard;
   }
   else
   FragColor = vec4(color.x, color.y, color.z, 1.0f);

}