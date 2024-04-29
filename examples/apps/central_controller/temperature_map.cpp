#include <iostream>
#include <unordered_map>
#include <array>
#include <sstream>
#include <iomanip>
#include <openthread/ip6.h>

static std::unordered_map<std::string, uint8_t> temperatureMap;

// Struct to hold IPv6 address and temperature
struct IPv6Temperature {
    std::array<uint8_t, 16> address; // IPv6 address as array of uint8_t
    uint8_t temperature; // Temperature
};

// Function to convert IPv6 address to string representation
std::string ipv6ToString(const std::array<uint8_t, 16>& address) {
    std::stringstream ss;
    for (const auto& byte : address) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    return ss.str();
}

// Function to retrieve temperature using IPv6 address from the hashmap
uint8_t getTemperature(const std::unordered_map<std::string, uint8_t>& temperatureMap, const std::array<uint8_t, 16>& address) {
    std::string ipv6String = ipv6ToString(address);
    auto it = temperatureMap.find(ipv6String);
    if (it != temperatureMap.end()) {
        return it->second;
    }
    return 0; // Return 0 if address not found
}

// Function to iterate through IPv6 unordered_map and sum the average temperatures
uint8_t averageTemperature(const std::unordered_map<std::string, uint8_t>& temperatureMap) {
	uint8_t sum = 0;
	for (const auto& entry : temperatureMap) {
		sum += entry.second;
	}
	return sum / temperatureMap.size();
}

extern "C" uint8_t otTemperatureInput(otIp6Address ipv6, uint8_t temperature)
{
	// create std::array<uint8_t, 16> from otIp6Address
	std::array<uint8_t, 16> ipv6Array;
	for (int i = 0; i < 16; i++) {
		ipv6Array[i] = ipv6.mFields.m8[i];
	}
	// store temperature in the hashmap
	temperatureMap[ipv6ToString(ipv6Array)] = temperature;

	// return average temperature
	return averageTemperature(temperatureMap);
}