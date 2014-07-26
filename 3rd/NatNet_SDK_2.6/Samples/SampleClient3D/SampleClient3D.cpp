//=============================================================================
// Copyright © 2009 NaturalPoint, Inc. All Rights Reserved.
// 
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall NaturalPoint, Inc. or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//=============================================================================


// NatNetSample.cpp : Defines the entry point for the application.
//
#ifdef WIN32
#  pragma warning( disable : 4996 )
#  define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
#endif

#include <cstring> // For memset.
#include <windows.h>
#include <winsock.h>
#include "resource.h"

#include <GL/gl.h>
#include <GL/glu.h>

//NatNet SDK
#include "NatNetTypes.h"
#include "NatNetClient.h"
#include "natutils.h"

#include "GLPrint.h"
#include "RigidBodyCollection.h"
#include "MarkerPositionCollection.h"
#include "OpenGLDrawingFunctions.h"

// globals
// Class for printing bitmap fonts in OpenGL
GLPrint glPrinter;


HINSTANCE hInst;

// OpenGL rendering context.
HGLRC openGLRenderContext = nullptr;

// Our NatNet client object.
NatNetClient natnetClient;

// Objects for saving off marker and rigid body data streamed
// from NatNet.
MarkerPositionCollection markerPositions;
RigidBodyCollection rigidBodies;

// Ready to render?
bool render = true;

// Used for converting NatNet data to the proper units.
float unitConversion = 1.0f;

// NatNet server IP address.
int IPAddress[4] = {127,0,0,1};

// Timecode string 
char szTimecode[128] = "";

// functions
BOOL InitInstance(HINSTANCE, int);
LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
LRESULT CALLBACK NatNetDlgProc(HWND, UINT, WPARAM, LPARAM);
void Render();
void DataHandler(sFrameOfMocapData* data, void* pUserData);			// receives data from the server
void MessageHandler(int msgType, char* msg);		// receives NatNet error mesages
bool InitNatNet(LPSTR szIPAddress, LPSTR szServerIPAddress);
bool ParseRigidBodyDescription(sDataDescriptions* pDataDefs);

//****************************************************************************
//
// Windows Functions 
//

// Register our window.
ATOM MyRegisterClass(HINSTANCE hInstance)
{
  WNDCLASSEX wcex;
  wcex.cbSize = sizeof(WNDCLASSEX); 
  wcex.style			= CS_HREDRAW | CS_VREDRAW;
  wcex.lpfnWndProc	= (WNDPROC)WndProc;
  wcex.cbClsExtra		= 0;
  wcex.cbWndExtra		= 0;
  wcex.hInstance		= hInstance;
  wcex.hIcon			= NULL;
  wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
  wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
  wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_NATNETSAMPLE);
  wcex.lpszClassName	= "NATNETSAMPLE";
  wcex.hIconSm		= NULL;
  return RegisterClassEx(&wcex);
}

// WinMain
int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
                     LPSTR lpCmdLine, int nCmdShow)
{
  MyRegisterClass(hInstance);

  if (!InitInstance (hInstance, nCmdShow)) 
    return false;

  MSG msg;

  while(true)
  {
    if(PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE))
    {
      if(!GetMessage(&msg, NULL, 0, 0))
        break;
      TranslateMessage(&msg);
      DispatchMessage(&msg);
    }
    else
    {
      if(render)
      {
        HDC hDC = GetDC(msg.hwnd);
        if(hDC)
        {
          wglMakeCurrent(hDC, openGLRenderContext);
          Render();
          SwapBuffers(hDC);
          wglMakeCurrent(0, 0);
        }
        ReleaseDC(msg.hwnd, hDC);
      }
    }
  }

  return (int)msg.wParam;
}

// Initialize new instances of our application
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
  hInst = hInstance;

  HWND hWnd = CreateWindow("NATNETSAMPLE", "NatNet Sample", WS_OVERLAPPEDWINDOW,
  CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);

  if (!hWnd)
    return false;

  // Define pixel format
  PIXELFORMATDESCRIPTOR pfd;
  int nPixelFormat;	
  memset(&pfd, NULL, sizeof(pfd));    
  pfd.nSize      = sizeof(pfd);
  pfd.nVersion   = 1;
  pfd.dwFlags    = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
  pfd.iPixelType = PFD_TYPE_RGBA;
  pfd.cColorBits = 24;
  pfd.cDepthBits = 16;
  pfd.iLayerType = PFD_MAIN_PLANE;

  // Set pixel format. Needed for drawing OpenGL bitmap fonts.
  HDC hDC = GetDC(hWnd);
  nPixelFormat = ChoosePixelFormat(hDC, &pfd);
  SetPixelFormat(hDC, nPixelFormat, &pfd);

  // Create and set the current OpenGL rendering context.
  openGLRenderContext = wglCreateContext(hDC);
  wglMakeCurrent(hDC, openGLRenderContext);

  // Set some OpenGL options.
  glClearColor(0.400f, 0.400f, 0.400f, 1.0f);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);

  // Set the device context for our OpenGL printer object.
  glPrinter.SetDeviceContext(hDC);


  wglMakeCurrent(0, 0);
  ReleaseDC(hWnd, hDC);

  ShowWindow(hWnd, nCmdShow);
  UpdateWindow(hWnd);

  // Make a good guess as to the IP address of our NatNet server.
  in_addr MyAddress[10];
  int nAddresses = NATUtils::GetLocalIPAddresses((unsigned long *)&MyAddress, 10);
  if (nAddresses > 0)
  {
    IPAddress[0] = MyAddress[0].S_un.S_un_b.s_b1;
    IPAddress[1] = MyAddress[0].S_un.S_un_b.s_b2;
    IPAddress[2] = MyAddress[0].S_un.S_un_b.s_b3;
    IPAddress[3] = MyAddress[0].S_un.S_un_b.s_b4;
  }

  return true;
}

// Windows message processing function.
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
  int wmId, wmEvent;
  PAINTSTRUCT ps;
  HDC hdc;

  switch (message) 
  {
  case WM_COMMAND:
    wmId    = LOWORD(wParam); 
    wmEvent = HIWORD(wParam); 
    // Parse the menu selections:
    switch (wmId)
    {
    case IDM_CONNECT:
      DialogBox(hInst, (LPCTSTR)IDD_NATNET, hWnd, (DLGPROC)NatNetDlgProc);
      break;
    case IDM_EXIT:
      DestroyWindow(hWnd);
      break;
    default:
      return DefWindowProc(hWnd, message, wParam, lParam);
    }

    break;

  case WM_PAINT:
    hdc = BeginPaint(hWnd, &ps);
    // Render a frame NatNet data. This data has been saved off in NatNet callbacks 
    // into the file level variables markerPositions and rigidBodies
    wglMakeCurrent(hdc, openGLRenderContext);
    Render();
    SwapBuffers(hdc);
    wglMakeCurrent(0, 0);
    EndPaint(hWnd, &ps);
    break;

  case WM_SIZE:
    {
      int cx = LOWORD(lParam), cy = HIWORD(lParam);
      if(cx != 0 && cy !=0 && hWnd != nullptr)
      {
        GLfloat fFovy  = 40.0f; // Field-of-view
        GLfloat fZNear = 1.0f;  // Near clipping plane
        GLfloat fZFar = 10000.0f;  // Far clipping plane

        HDC hDC = GetDC(hWnd);
        wglMakeCurrent(hDC, openGLRenderContext);

        // Calculate OpenGL viewport aspect
        RECT rv;
        GetClientRect(hWnd, &rv);
        GLfloat fAspect = (GLfloat)(rv.right-rv.left) / (GLfloat)(rv.bottom-rv.top);

        // Define OpenGL viewport
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(fFovy, fAspect, fZNear, fZFar);
        glViewport(rv.left, rv.top, rv.right-rv.left, rv.bottom-rv.top);
        glMatrixMode(GL_MODELVIEW);

        wglMakeCurrent(0, 0);
        ReleaseDC(hWnd, hDC);
      }
    }
    break;

  case WM_DESTROY:
    {
      HDC hDC = GetDC(hWnd);
      wglMakeCurrent(hDC, openGLRenderContext);
      natnetClient.Uninitialize();
      wglMakeCurrent(0, 0);
      wglDeleteContext(openGLRenderContext);
      ReleaseDC(hWnd, hDC);
      PostQuitMessage(0);
    }
    break;

  default:
    return DefWindowProc(hWnd, message, wParam, lParam);
  }

  return 0;
}

// Main OpenGL rendering function. 
void Render()
{
  GLfloat m[9];
  GLfloat v[3];
  float fRadius = 5.0f;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear buffers
  glLoadIdentity(); // Load identity matrix

  GLfloat glfLight[] = {-4.0f, 4.0f, 4.0f, 0.0f};
  glLightfv(GL_LIGHT0, GL_POSITION, glfLight);

  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

  glPushMatrix();

  // draw timecode
  glPushMatrix();
  glTranslatef(2400.f, -1750.f, -5000.0f);	
  glPrinter.Print(0.0f,0.0f,szTimecode);
  glPopMatrix();

  // Position and rotate the camera
  glTranslatef(0.0f, -1000.0f, -5000.0f);	

  //draw axis
  glBegin(GL_LINES);
  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3f(0,0,0);
  glVertex3f(300,0,0);

  glColor3f(0.0f, 1.0f, 0.0f);
  glVertex3f(0,0,0);
  glVertex3f(0,300,0);

  glColor3f(0.0f, 0.0f, 1.0f);
  glVertex3f(0,0,0);
  glVertex3f(0,0,300);
  glEnd();

  // draw grid
  OpenGLDrawingFunctions::DrawGrid();

  // Draw the rigid bodies
  for (size_t i = 0; i < rigidBodies.Count(); i++)
  {
    GLfloat x, y, z;
    std::tie(x, y, z) = rigidBodies.GetCoordinates(i);
    x *= unitConversion;	// convert to mm
    y *= unitConversion;
    z *= unitConversion;
    GLfloat qx, qy, qz, qw;
    std::tie(qx, qy, qz, qw) = rigidBodies.GetQuaternion(i);
    glColor4f(0.0f,0.0f,1.0f,1.0f);
    OpenGLDrawingFunctions::DrawBox(x,y,z,qx,qy,qz,qw);
    glColor4f(0.0f,0.0f,0.0f,1.0f);

    // Convert quaternion to eulers.  Motive coordinate conventions: X(Pitch), Y(Yaw), Z(Roll), Relative, RHS
    Quat q;
    q.x = qx; q.y = qy; q.z = qz; q.w = qw;
    int order = EulOrdXYZr;
    EulerAngles ea = Eul_FromQuat(q, order);
    ea.x = NATUtils::RadiansToDegrees(ea.x);
    ea.y = NATUtils::RadiansToDegrees(ea.y);
    ea.z = NATUtils::RadiansToDegrees(ea.z);
    glPrinter.Print(x,y,"RB %d (Pitch: %3.1f, Yaw: %3.1f, Roll: %3.1f)", rigidBodies.ID(i), ea.x, ea.y, ea.z);
  }

  // draw markers
  // [optional] local coordinate support : get first rb's pos/ori (face support only- assume 1st is root)
  if(rigidBodies.Count()==1)
  {
    GLfloat q[4];
    std::tie(q[0], q[1], q[2], q[3]) = rigidBodies.GetQuaternion(0);
    NATUtils::QaternionToRotationMatrix(q, m);
  }

  for (size_t i = 0; i < markerPositions.MarkerPositionCount(); i++)
  {
    std::tie(v[0], v[1], v[2]) = markerPositions.GetMarkerPosition(i);

    // [optional] local coordinate support : inherit (accumulate) parent's RB pos/ori ("root") if using local marker position
    if(rigidBodies.Count()==1)
    {
      NATUtils::Vec3MatrixMult(v,m);
      v[0] += std::get<0>(rigidBodies.GetCoordinates(0));
      v[1] += std::get<1>(rigidBodies.GetCoordinates(0));
      v[2] += std::get<2>(rigidBodies.GetCoordinates(0));
    }
    glPushMatrix();
    glTranslatef(v[0] * unitConversion, v[1] * unitConversion, v[2] * unitConversion);
    OpenGLDrawingFunctions::DrawSphere(1,fRadius);
    glPopMatrix();
  }

  //draw labeled markers
  for (size_t i = 0; i < markerPositions.LabeledMarkerPositionCount(); i++)
  {
    const sMarker& markerData = markerPositions.GetLabeledMarker(i);
    v[0] = markerData.x * unitConversion;
    v[1] = markerData.y * unitConversion;
    v[2] = markerData.z * unitConversion;
    fRadius = markerData.size * unitConversion;
    glPushMatrix();
    glTranslatef(v[0],v[1],v[2]);
    glColor4f(1.0f,0.0f,0.0f,1.0f);
    OpenGLDrawingFunctions::DrawSphere(1,fRadius);
    glPopMatrix();
  }

  glPopMatrix();

  glFlush();
  // Done rendering a frame. The NatNet callback function DataHandler
  // will set render to true when it receives another frame of data.
  render = false; 
}

// Callback for the connect-to-NatNet dialog. Gets the server and local IP 
// addresses and attempts to initialize the NatNet client.
LRESULT CALLBACK NatNetDlgProc(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
  char szBuf[512];
  switch (message)
  {
  case WM_INITDIALOG:
    SetDlgItemText(hDlg, IDC_EDIT1, _itoa(IPAddress[0], szBuf,10));
    SetDlgItemText(hDlg, IDC_EDIT2, _itoa(IPAddress[1], szBuf,10));
    SetDlgItemText(hDlg, IDC_EDIT3, _itoa(IPAddress[2], szBuf,10));
    SetDlgItemText(hDlg, IDC_EDIT4, _itoa(IPAddress[3], szBuf,10));
    SetDlgItemText(hDlg, IDC_EDIT5, _itoa(IPAddress[0], szBuf,10));
    SetDlgItemText(hDlg, IDC_EDIT6, _itoa(IPAddress[1], szBuf,10));
    SetDlgItemText(hDlg, IDC_EDIT7, _itoa(IPAddress[2], szBuf,10));
    SetDlgItemText(hDlg, IDC_EDIT8, _itoa(IPAddress[3], szBuf,10));
    return true;

  case WM_COMMAND:
    switch (LOWORD(wParam))
    {
    case IDC_CONNECT:
      {
        char szMyIPAddress[30], szServerIPAddress[30];
        char ip1[5], ip2[5], ip3[5], ip4[5];
        GetDlgItemText(hDlg, IDC_EDIT1, ip1, 4);
        GetDlgItemText(hDlg, IDC_EDIT2, ip2, 4);
        GetDlgItemText(hDlg, IDC_EDIT3, ip3, 4);
        GetDlgItemText(hDlg, IDC_EDIT4, ip4, 4);
        sprintf_s(szMyIPAddress, 30, "%s.%s.%s.%s", ip1,ip2,ip3,ip4);

        GetDlgItemText(hDlg, IDC_EDIT5, ip1, 4);
        GetDlgItemText(hDlg, IDC_EDIT6, ip2, 4);
        GetDlgItemText(hDlg, IDC_EDIT7, ip3, 4);
        GetDlgItemText(hDlg, IDC_EDIT8, ip4, 4);
        sprintf_s(szServerIPAddress, 30, "%s.%s.%s.%s", ip1,ip2,ip3,ip4);

        // Try and intialize the NatNet client.
        if (InitNatNet(szMyIPAddress, szServerIPAddress) == false)
        {
          natnetClient.Uninitialize();
          MessageBox(hDlg, "Failed to connect", "", MB_OK);
        }
      }
    case IDOK:
    case IDCANCEL:
      EndDialog(hDlg, LOWORD(wParam));
      return true;
    }
  }
  return false;
}

// Initialize the NatNet client with client and server IP addresses.
bool InitNatNet(LPSTR szIPAddress, LPSTR szServerIPAddress)
{
  unsigned char ver[4];
  natnetClient.NatNetVersion(ver);

  // Set callback handlers
  // Callback for NatNet messages.
  natnetClient.SetMessageCallback(MessageHandler);
  // NatNet verbosity level. We want none.
  natnetClient.SetVerbosityLevel(Verbosity_None);
  // this function will receive data from the server
  natnetClient.SetDataCallback( DataHandler );	

  int retCode = natnetClient.Initialize(szIPAddress, szServerIPAddress);
  if (retCode != ErrorCode_OK)
  {
    //Unable to connect to server.
    return false;
  }
  else
  {
    // Print server info
    sServerDescription ServerDescription;
    memset(&ServerDescription, 0, sizeof(ServerDescription));
    natnetClient.GetServerDescription(&ServerDescription);
    if(!ServerDescription.HostPresent)
    {
      //Unable to connect to server. Host not present
      return false;
    }
  }

  // Retrieve RigidBody description from server
  sDataDescriptions* pDataDefs = NULL;
  int nBodies = natnetClient.GetDataDescriptions(&pDataDefs);
  if (ParseRigidBodyDescription(pDataDefs) == false)
  {
    //Unable to retrieve RigidBody description
    //return false;
  }

  // example of NatNet general message passing. Set units to millimeters
  // and get the multiplicative conversion factor in the response.
  void* response;
  int nBytes;
  retCode = natnetClient.SendMessageAndWait("UnitsToMillimeters", &response, &nBytes);
  if (retCode == ErrorCode_OK)
  {
    unitConversion = *(float*)response;
  }

  return true;
}

bool ParseRigidBodyDescription(sDataDescriptions* pDataDefs)
{
  if (pDataDefs == NULL || pDataDefs->nDataDescriptions <= 0)
    return false;

  for (int i=0,j=0;i<pDataDefs->nDataDescriptions;i++)
  {
    if (pDataDefs->arrDataDescriptions[i].type != Descriptor_RigidBody)
      continue;
    
    sRigidBodyDescription *pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
    //construct skeleton hierarchy
    int ID        = pRB->ID;
    int parentID  = pRB->parentID;
    float offsetx = pRB->offsetx;
    float offsety = pRB->offsety;
    float offsetz = pRB->offsetz;
  }

  return true;
}

// Handler for NatNet messages. 
void MessageHandler(int msgType, char* msg)
{
  //	printf("\n[SampleClient] Message received: %s\n", msg);
}

// NatNet data callback function. Stores rigid body and marker data in the file level 
// variables markerPositions, and rigidBodies and sets the file level variable render
// to true. This signals that we have a frame ready to render.
void DataHandler(sFrameOfMocapData* data, void* pUserData)
{
  int mcount = min(MarkerPositionCollection::MAX_MARKER_COUNT,data->MocapData->nMarkers);
  markerPositions.SetMarkerPositions(data->MocapData->Markers, mcount);

  // unidentified markers
  mcount = min(MarkerPositionCollection::MAX_MARKER_COUNT,data->nOtherMarkers);
  markerPositions.AppendMarkerPositions(data->OtherMarkers, mcount);

  int rbcount = min(RigidBodyCollection::MAX_RIGIDBODY_COUNT, data->nRigidBodies);
  rigidBodies.SetRigidBodyData(data->RigidBodies, rbcount);

  for (int s = 0; s < data->nSkeletons; s++)
  {
    rigidBodies.AppendRigidBodyData(data->Skeletons[s].RigidBodyData, data->Skeletons[s].nRigidBodies);
  }

  markerPositions.SetLabledMarkers(data->LabeledMarkers, data->nLabeledMarkers);

  // timecode
  NatNetClient* pClient = (NatNetClient*)pUserData;
  // decode to values
  int hour, minute, second, frame, subframe;
  bool bValid = pClient->DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
  // decode to friendly string
  pClient->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);

  render = true;
}
