/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Module:       FGOutputJsonSocket.cpp
 Author:       Carlos Figueroa; Bertrand Coconnier
 Date started: 09/10/11
 Purpose:      Manage output of sim parameters to a socket
 Called by:    FGOutput

 ------------- Copyright (C) 2011 Bertrand Coconnier -------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free Software
 Foundation; either version 2 of the License, or (at your option) any later
 version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along with
 this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 Place - Suite 330, Boston, MA  02111-1307, USA.

 Further information about the GNU Lesser General Public License can also be found on
 the world wide web at http://www.gnu.org.

FUNCTIONAL DESCRIPTION
--------------------------------------------------------------------------------
This is the place where you create output routines to dump data for perusal
later.

HISTORY
--------------------------------------------------------------------------------
09/10/11   BC    Created

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
INCLUDES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <cstring>
#include <cstdlib>

#include "FGOutputJsonSocket.h"
#include "FGFDMExec.h"
#include "models/FGAerodynamics.h"
#include "models/FGAccelerations.h"
#include "models/FGAircraft.h"
#include "models/FGAtmosphere.h"
#include "models/FGAuxiliary.h"
#include "models/FGPropulsion.h"
#include "models/FGMassBalance.h"
#include "models/FGPropagate.h"
#include "models/FGGroundReactions.h"
#include "models/FGFCS.h"
#include "models/atmosphere/FGWinds.h"
#include "input_output/FGXMLElement.h"
#include "math/FGPropertyValue.h"

using namespace std;

namespace JSBSim {

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS IMPLEMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

FGOutputJsonSocket::FGOutputJsonSocket(FGFDMExec* fdmex) :
  FGOutputType(fdmex),
  socket(0), SockProtocol (FGfdmSocket::ptUDP)
{
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FGOutputJsonSocket::~FGOutputJsonSocket()
{
  delete socket;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGOutputJsonSocket::SetOutputName(const string& fname)
{
  // tokenize the output name
  size_t dot_pos = fname.find(':', 0);
  size_t slash_pos = fname.find('/', 0);
  
  string name = fname.substr(0, dot_pos);
  
  string proto = "UDP";
  if(dot_pos + 1 < slash_pos)
    proto = fname.substr(dot_pos + 1, slash_pos - dot_pos - 1);
  
  string port = "1138";
  if(slash_pos < string::npos)
    port = fname.substr(slash_pos + 1, string::npos);
  
  // set the model name
  Name = name + ":" + port + "/" + proto;
  
  // set the socket params
  SockName = name;
  
  SockPort = atoi(port.c_str());
  
  if (to_upper(proto) == "UDP")
    SockProtocol = FGfdmSocket::ptUDP;
  /*else // Default to TCP
    SockProtocol = FGfdmSocket::ptTCP;*/
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool FGOutputJsonSocket::Load(Element* el)
{
  if (!FGOutputType::Load(el))
    return false;

  SetOutputName(el->GetAttributeValue("name") + ":" +
                el->GetAttributeValue("protocol") + "/" +
                el->GetAttributeValue("port"));

  return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool FGOutputJsonSocket::InitModel(void)
{
  if (FGOutputType::InitModel()) {
    delete socket;
    socket = new FGfdmSocket(SockName, SockPort, SockProtocol);

    if (socket == 0) return false;
    if (!socket->GetConnectStatus()) return false;

    PrintHeaders();

    return true;
  }

  return false;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGOutputJsonSocket::PrintHeaders(void)
{

  /*if (SubSystems & ssAeroFunctions) {
    scratch = Aerodynamics->GetAeroFunctionStrings(",");
    if (scratch.length() != 0) socket->Append(scratch);
  }

  if (SubSystems & ssFCS) {
    scratch = FCS->GetComponentStrings(",");
    if (scratch.length() != 0) socket->Append(scratch);
  }

  if (SubSystems & ssGroundReactions)
    socket->Append(GroundReactions->GetGroundReactionStrings(","));

  if (SubSystems & ssPropulsion && Propulsion->GetNumEngines() > 0)
    socket->Append(Propulsion->GetPropulsionStrings(","));

  for (unsigned int i=0;i<OutputParameters.size();++i) {
    if (!OutputCaptions[i].empty())
      socket->Append(OutputCaptions[i]);
    else
      socket->Append(OutputParameters[i]->GetPrintableName());
  }*/
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGOutputJsonSocket::Print(void)
{
	string asciiData, scratch;
	stringstream json;

  if (socket == 0) return;
  if (!socket->GetConnectStatus()) return;
  if (FCS->GetTrimStatus()) return;

  json.clear();
  json << "{\n \"JSBSim_";
  json << Aircraft->GetName();
  json << "\": {\n";
  json << "\"timestamp\" : ";
  json << FDMExec->GetSimTime();

  if (SubSystems & ssAerosurfaces) {
	  json << ",\n";
	  json << "\n\"ControlSurfaces\": { ";
	  json << "\"AileronCommand\": ";
	  json << FCS->GetDaCmd();
	  json << ", \"ElevatorCommand\": ";
	  json << FCS->GetDeCmd();
	  json << ", \"RudderCommand\": ";
	  json << FCS->GetDrCmd();
	  json << ", \"FlapCommand\": ";
      json << FCS->GetDfCmd();
	  json << ", \"LeftAileronPosition\": ";
      json << FCS->GetDaLPos();
	  json << ", \"RightAileronPosition\": ";
      json << FCS->GetDaRPos();
	  json << ", \"ElevatorPosition\": ";
      json << FCS->GetDePos();
	  json << ", \"RudderPosition\": ";
      json << FCS->GetDrPos();
	  json << ", \"FlapPosition\": ";
      json << FCS->GetDfPos();
	  json << " }";
  }
  if (SubSystems & ssRates) {
	  json << ",\n";
	  json << "\"Rates\": { ";
	  json << "\"P\": ";
	  json << radtodeg*Propagate->GetPQR(eP);
	  json << ", \"Q\": ";
	  json << radtodeg*Propagate->GetPQR(eQ);
	  json << ", \"R\": ";
	  json << radtodeg*Propagate->GetPQR(eR);
	  json << ", \"PDot\": ";
      json << radtodeg*Accelerations->GetPQRdot(eP);
	  json << ", \"QDot\": ";
	  json << radtodeg*Accelerations->GetPQRdot(eQ);
	  json << ", \"RDot\": ";
	  json << radtodeg*Accelerations->GetPQRdot(eR);
	  json << " }";
  }
  if (SubSystems & ssVelocities) {
	  json << ",\n";
	  json << "\"Velocities\": { ";
	  json << "\"QBar\": ";
	  json << Auxiliary->Getqbar();
	  json << ", \"Vtotal\": ";
      json << Auxiliary->GetVt();
	  json << ", \"UBody\": ";
      json << Propagate->GetUVW(eU);
	  json << ", \"VBody\": ";
      json << Propagate->GetUVW(eV);
	  json << ", \"WBody\": ";
      json << Propagate->GetUVW(eW);
	  json << ", \"UAero\": ";
      json << Auxiliary->GetAeroUVW(eU);
	  json << ", \"VAero\": ";
      json << Auxiliary->GetAeroUVW(eV);
	  json << ", \"WAero\": ";
      json << Auxiliary->GetAeroUVW(eW);
	  json << ", \"Vn\": ";
      json << Propagate->GetVel(eNorth);
	  json << ", \"Ve\": ";
      json << Propagate->GetVel(eEast);
	  json << ", \"Vd\": ";
      json << Propagate->GetVel(eDown);
	  json << " }";
  }

 if (SubSystems & ssForces) {
	  json << ",\n";
	  json << "\"Forces\": { ";
	  json << "\"F_Drag\": ";
	  json << Aerodynamics->GetvFw()(eDrag);
	  json << ", \"F_Side\": ";
      json << Aerodynamics->GetvFw()(eSide);
	  json << ", \"F_Lift\": ";
      json << Aerodynamics->GetvFw()(eLift);
	  json << ", \"LoD\": ";
      json << Aerodynamics->GetLoD();
	  json << ", \"Fx\": ";
      json << Aircraft->GetForces(eX);
	  json << ", \"Fy\": ";
      json << Aircraft->GetForces(eY);
	  json << ", \"Fz\": ";
      json << Aircraft->GetForces(eZ);
	  json << " }";
  }
  if (SubSystems & ssMoments) {
	  json << ",\n";
	  json << "\"Moments\": { ";
	  json << "\"L\": ";
	  json << Aircraft->GetMoments(eL);
	  json << ", \"M\": ";
      json << Aircraft->GetMoments(eM);
	  json << ", \"N\": ";
      json << Aircraft->GetMoments(eN);
	  json << " }";
  }
   if (SubSystems & ssAtmosphere) {
	json << ",\n";
	json << "\"Atmosphere\": { ";
	json << "\"Rho\": ";
	json << Atmosphere->GetDensity();
	json << ", \"SL_pressure\": ";
    json << Atmosphere->GetPressureSL();
	json << ", \"Ambient_pressure\": ";
    json << Atmosphere->GetPressure();
	json << ", \"Turbulence_Magnitude\": ";
    json << Winds->GetTurbMagnitude();
	json << ", \"Turbulence_Direction\": ";
    json << Winds->GetTurbDirection();
	json << ", \"NWind\": ";
	json << Winds->GetTotalWindNED(eNorth);
	json << ", \"EWind\": ";
	json << Winds->GetTotalWindNED(eEast);
	json << ", \"DWind\": ";
	json << Winds->GetTotalWindNED(eDown);
	json << " }";
  }
  if (SubSystems & ssMassProps) {
	json << ",\n";
	json << "\"MassProperties\": { ";
	json << "\"Ixx\": ";
	json << MassBalance->GetJ()(1,1);
	json << ", \"Ixy\": ";
    json << MassBalance->GetJ()(1,2);
	json << ", \"Ixz\": ";
    json << MassBalance->GetJ()(1,3);
	json << ", \"Iyx\": ";
    json << MassBalance->GetJ()(2,1);
	json << ", \"Iyy\": ";
    json << MassBalance->GetJ()(2,2);
	json << ", \"Iyz\": ";
    json << MassBalance->GetJ()(2,3);
	json << ", \"Izx\": ";
    json << MassBalance->GetJ()(3,1);
	json << ", \"Izy\": ";
    json << MassBalance->GetJ()(3,2);
	json << ", \"Izz\": ";
    json << MassBalance->GetJ()(3,3);
	json << ", \"Mass\": ";
    json << MassBalance->GetMass();
	json << ", \"Xcg\": ";
    json << MassBalance->GetXYZcg()(eX);
	json << ", \"Ycg\": ";
    json << MassBalance->GetXYZcg()(eY);
	json << ", \"Zcg\": ";
    json << MassBalance->GetXYZcg()(eZ);
	json << " }";
  }
  if (SubSystems & ssPropagate) {
	json << ",\n";
	json << "\"Propagate\": { ";
	json << "\"Altitude\": ";
	json << Propagate->GetAltitudeASL();
	json << ", \"Phi_(deg)\": ";
    json << radtodeg*Propagate->GetEuler(ePhi);
	json << ", \"Theta_(deg)\": ";
    json << radtodeg*Propagate->GetEuler(eTht);
	json << ", \"Psi_(deg)\": ";
    json << radtodeg*Propagate->GetEuler(ePsi);
	json << ", \"Alpha_(deg)\": ";
    json << Auxiliary->Getalpha(inDegrees);
	json << ", \"Beta_(deg)\": ";
    json << Auxiliary->Getbeta(inDegrees);
	json << ", \"Latitude_(deg)\": ";
    json << Propagate->GetLocation().GetLatitudeDeg();
	json << ", \"Longitude_(deg)\": ";
    json << Propagate->GetLocation().GetLongitudeDeg();
	json << " }";
  }
  /*if (SubSystems & ssAeroFunctions) {
    scratch = Aerodynamics->GetAeroFunctionValues(",");
    if (scratch.length() != 0) socket->Append(scratch);
  }
  if (SubSystems & ssFCS) {
    scratch = FCS->GetComponentValues(",");
    if (scratch.length() != 0) socket->Append(scratch);
  }
  if (SubSystems & ssGroundReactions) {
    socket->Append(GroundReactions->GetGroundReactionValues(","));
  }
  if (SubSystems & ssPropulsion && Propulsion->GetNumEngines() > 0) {
    socket->Append(Propulsion->GetPropulsionValues(","));
  }

  for (unsigned int i=0;i<OutputParameters.size();++i) {
    socket->Append(OutputParameters[i]->GetValue());
  }*/

  json << "\n}\n}";

  socket->Send(json.str().c_str(), json.str().length());
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGOutputJsonSocket::SocketStatusOutput(const string& out_str)
{
}
}
