#ifdef _WIN32
#define NOMINMAX
#endif

#include "../include/gameWorld.hpp"

int main(int argc, char* argv[])
{
	GameWorld gameWorld;
	gameWorld.run();
	return EXIT_SUCCESS;
}