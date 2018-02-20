#ifndef PEGASUS_DEBUG_HPP
#define PEGASUS_DEBUG_HPP

#include <pegasus/Collision.hpp>
#include <functional>

#define PEGASUS_DEBUG

namespace pegasus
{
namespace debug
{

class Debug
{
public:
	Debug() = default;
	Debug& operator==(Debug const&) = delete;
	Debug(Debug&) = delete;
	Debug& operator==(Debug&&) = delete;
	Debug(Debug&&) = delete;

	static Debug& GetInstace()
	{
		static Debug debug;
		return debug;
	}

	static void CollisionDetectionCall(std::vector<std::vector<collision::Contact>>& contacts)
#ifdef PEGASUS_DEBUG
	{
		Debug& debug = GetInstace();

        if (debug.collisionDetectionCall)
		{
            debug.collisionDetectionCall(contacts);
		}
	}
#else
	{
	}
#endif

	std::function<void(std::vector<std::vector<collision::Contact>>&)> collisionDetectionCall;
};

} // debug
} // pegasus

#endif // PEGASUS_DEBUG_HPP
