#include "main.h"
#include "subsystems/pneumatics.hpp"

int kPneumaticScraperPort = 1; // HIGH - down
int kPneumaticDescorePort = 4; // HIGH - arm out for descoring
int kPneumaticGatePort = 3; // HIGH - gate open for scoring
// int kPneumaticIntakePort = 3; // HIGH - 2 bar up for low goal
int kPneumaticMiddleDescorePort = 2; // HIGH - descore up for middle goal scoring
