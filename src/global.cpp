#include "vex.h"

using namespace vex;

class global
{
public:
  char *missingConnections(char *deviceGroupName, device devices[], char *deviceNames[]);

  // Returns a description of missing connections from the input devices
  static char *global::missingConnections(char *deviceGroupName, device devices[], char *deviceNames[])
  {
    char *missingConnections = "Missing ";
    strcat(missingConnections, deviceGroupName);
    strcat(missingConnections, " Connections: ");

    for (int i = 0; i < sizeof(devices); i++)
    {
      if (!devices[i].installed())
      {
        strcat(missingConnections, deviceNames[i]);
        strcat(missingConnections, ", ");
      }
    }

    if (strcmp(missingConnections, "Missing connections: ") == 0)
    {
      return strcat(deviceGroupName, " fully connected!");
    }

    // remove trailing comma and space
    missingConnections[strlen(missingConnections) - 2] = '\0';
    return missingConnections;
  }
};