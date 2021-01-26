/*	@file		OpenNR-IF.h
 *	@brief		OpenNR Interface
 *	@version	V1.4
 */
typedef void* NACHI_HANDLE;

#pragma once

#include <stddef.h>
#include <string>

#define NR_EXTENDED_DLL_API

//================================================
// Define
//================================================

//------------------------------------------------
// CONNECT SPEC
//------------------------------------------------
#define		NR_OBJECT_INTERNAL		(0)				// Internal 
#define		NR_OBJECT_EXTERNAL_TCP	(1)				// External(TCP)
#define		NR_OBJECT_EXTERNAL_UDP	(2)				// External(UDP)

#define		NR_MODE_COMM_SERVER		(0)				// Server For Communication 
#define		NR_MODE_COMM_CLIENT		(1)				// Client For Communication

#define		NR_DATA_REAL			(0)				// Server For Communication 
#define		NR_DATA_XML				(1)				// Client For Communication

//------------------------------------------------
// DATA ACCESS MODE
//------------------------------------------------

#define		NR_ACCESS_WAIT			0				// Wait
#define		NR_ACCESS_NO_WAIT		1				// No Wait?j

//------------------------------------------------
// Run/Stop
//------------------------------------------------

// For CtrlMotor
#define		NR_STANBY_OFF_REQ		0				// Stanby OFF
#define		NR_STANBY_ON_REQ		1				// Stanby ON

// For CtrlRun
#define		NR_STOP_REQ				0				// Stop
#define		NR_RUN_REQ				1				// Run

// For CtrlEventLog / CtrlMakeDirectory
#define		NR_INTERNAL_REQ			0				// Internal Memory
#define		NR_RC_EXTERNAL_REQ		1				// RC External Memory

// For CtrlEventLog
#define		NR_EVENT_LOG_SYSTEM			0			// System
#define		NR_EVENT_LOG_APPLICATION	1			// Application
#define		NR_EVENT_LOG_SECURITY		2			// Security

// For NR_AcsFTP
// FTP service start/stop
#define		NR_FTP_SERVICE_START		1
#define		NR_FTP_SERVICE_STOP			0

// FTP service ANONYMOUS
#define		NR_FTP_ANONYMOUS_FORBID			0
#define		NR_FTP_ANONYMOUS_ALLOW			1
#define		NR_FTP_ANONYMOUS_ONLY_ALLOW		2

// FTP service Connect Num
#define		NR_FTP_CONNECT_NUM_MIN			0
#define		NR_FTP_CONNECT_NUM_MAX			10

// FTP service Timeout
#define		NR_FTP_TIMEOUT_MIN				0
#define		NR_FTP_TIMEOUT_MAX				900

// FTP service Directory State
#define		NR_FTP_DIRECTORY_ONLY_READ		0
#define		NR_FTP_DIRECTORY_ONLY_WRITE		1
#define		NR_FTP_DIRECTORY_READ_WRITE		2

//------------------------------------------------
// Real Data ID
//------------------------------------------------

// For Get()------EXT
typedef	enum NR_GetRealDataExtId
{
  eNR_GET_STATUS_EXT_ID = 0,
  eNR_GET_TIME_EXT_ID,
  eNR_GET_CUR_TCP_POS_EXT_ID,
  eNR_GET_COM_TCP_POS_EXT_ID,
  eNR_GET_WORK_COORD_EXT_ID,
  eNR_GET_TOOL_COORD_EXT_ID,
  eNR_GET_CUR_ANGLE_EXT_ID,
  eNR_GET_COM_ANGLE_EXT_ID,
  eNR_GET_TORQUE_EXT_ID,
  eNR_GET_MORTOR_AMP_EXT_ID,
  eNR_GET_DIG_OUT_EXT_ID,
  eNR_GET_DIG_IN_EXT_ID,
  eNR_GET_ANA_OUT_EXT_ID,
  eNR_GET_ANA_IN_EXT_ID,
  eNR_GET_REAL_MAX_EXT_ID
} NR_GET_REAL_DATA_EXT_EXT_ID;

// For Set()------EXT
typedef	enum NR_SetRealDataExtId
{
  eNR_SET_STATUS_EXT_ID = 0,
  eNR_SET_TIME_EXT_ID,
  eNR_SET_COM_TCP_POS_EXT_ID,
  eNR_SET_WORK_COORD_EXT_ID,
  eNR_SET_TOOL_COORD_EXT_ID,
  eNR_SET_COM_ANGLE_EXT_ID,
  eNR_SET_DIG_OUT_EXT_ID,
  eNR_SET_ANA_OUT_EXT_ID,
  eNR_SET_REAL_MAX_EXT_ID
} NR_SET_REAL_DATA_EXT_EXT_ID;

// For Get()------STD
typedef	enum NR_GetRealDataStdId
{
  eNR_GET_STATUS_STD_ID = 0,
  eNR_GET_TIME_STD_ID,
  eNR_GET_CUR_TCP_POS_STD_ID,
  eNR_GET_CUR_ANGLE_STD_ID,
  eNR_GET_MORTOR_AMP_STD_ID,
  eNR_GET_DIG_OUT_STD_ID,
  eNR_GET_DIG_IN_STD_ID,
  eNR_GET_REAL_MAX_STD_ID
} NR_GET_REAL_DATA_STD_STD_ID;

// For Set()------STD
typedef	enum NR_SetRealDataStdId
{
  eNR_SET_STATUS_STD_ID = 0,
  eNR_SET_TIME_STD_ID,
  eNR_SET_COM_TCP_POS_STD_ID,
  eNR_SET_WORK_COORD_STD_ID,
  eNR_SET_TOOL_COORD_STD_ID,
  eNR_SET_COM_ANGLE_STD_ID,
  eNR_SET_DIG_OUT_STD_ID,
  eNR_SET_REAL_MAX_STD_ID
} NR_SET_REAL_DATA_STD_STD_ID;


//------------------------------------------------
// SubId MAX?i For Set()/Get()?j
//------------------------------------------------
#define		NR_MAX_AXIS_STD			(8)
#define		NR_MAX_AXIS				(7)
#define		NR_MAX_XYZRPY			(6)
#define		NR_MAX_DIGITAL_SIG		(64)
#define		NR_MAX_ANALOG_CH		(16)

//------------------------------------------------
// Priority
//------------------------------------------------
#define		NR_PULL_MODE			(-1)
#define		NR_PUSH_MODE_5			(0)
#define		NR_PUSH_MODE_10			(1)
#define		NR_PUSH_MODE_50			(2)
#define		NR_PUSH_MODE_100		(3)
#define		NR_PUSH_MODE_200		(4)
#define		NR_PUSH_MODE_500		(5)
#define		NR_PUSH_MODE_1000		(6)

//================================================
// I/F RET CODE
//================================================

#define		NR_E_NORMAL				0					// Normal

#define		NR_E_ALREADY			1001				// Alrady
#define		NR_E_NONE_SEND			1002				// None Send (Alrady Send)
#define		NR_E_EXIT				1003				// Process Exit
//------------------------------------------------
// I/F ERROR
//------------------------------------------------
#define		NR_E_PARAM				-1001				// I/F Parameter Error
#define		NR_E_INVALID			-1002				// I/F Invalid Error
#define		NR_E_NONE_SUPPORT		-1003				// I/F None Support
#define		NR_E_EXEC_FAIL			-1004				// Exec Error 
#define		NR_E_SEQ				-1005				// Sequence Error
#define		NR_E_LICENSE			-1006				// LICENSE Error
#define		NR_E_XML_SCHEMA			-1007				// XML schema Error
#define		NR_E_VERSION_NONE_SUPPORT -1010				// VERSION NONE SUPPORT

//------------------------------------------------
// I/F CONNECT ERROR
//------------------------------------------------
#define		NR_E_SEND				-2001				// Send Error
#define		NR_E_SEND_TIME_OUT		-2002				// Response Recv TimeOut After Send Error
#define		NR_E_RECV				-2003				// Recv Error
#define		NR_E_RETRY_OVER			-2004				// Retry Over
#define		NR_E_RECV_TIME_OUT		-2005				// Recv TimeOut

//------------------------------------------------
// I/F UPDATE ERROR
//------------------------------------------------
#define		NR_E_UDATE_FATAL		-3001				// fatal error
#define		NR_E_UDATE_READONLY		-3002				// read only
#define		NR_E_UDATE_MISMATCH		-3003				// mode	mismatch
#define		NR_E_UDATE_MISSING		-3004				// missing data
#define		NR_E_UDATE_UNITINVALID	-3005				// unit invalid

//------------------------------------------------
// I/F CTRL ERROR
//------------------------------------------------
#define		NR_E_CTRL_FATAL			-4001				// fatal error
#define		NR_E_CTRL_FORMAT		-4002				// format error
#define		NR_E_CTRL_LIMIT			-4003				// limit error
#define		NR_E_CTRL_EXECFAIL		-4004				// exec	error

//------------------------------------------------
// Pack data ID
//------------------------------------------------
typedef	enum NR_SetPackDataId
{
  eNR_SET_NONE = 0,
  eNR_SET_ENC_CUR_ID,					// Encoder   (Current)
  eNR_SET_ENC_COM_ID,					// Encoder   (Command)
  eNR_SET_POS_CUR_ID,					// Position  (Current)
  eNR_SET_POS_COM_ID,					// Position  (Command)
  eNR_SET_CURR_CUR_ID,				// Current   (Current)
  eNR_SET_CURR_COM_ID,				// Current   (Command)
  eNR_SET_SPD_CUR_ID,					// Speed     (Current)
  eNR_SET_SPD_COM_ID,					// Speed     (Command)
  eNR_SET_TCP_CUR_ID,					// TCP pos.  (Current)
  eNR_SET_TCP_COM_ID,					// TCP pos.  (Command)
  eNR_SET_TCPSPD_CUR_ID,				// TCP Speed (Current)
  eNR_SET_TCPSPD_COM_ID,				// TCP Speed (Command)
  eNR_SET_UBTRQ_ID,					// Unbalance torque
  eNR_SET_FIXIN_ID,					// Fixed input
  eNR_SET_FIXOUT_ID,					// Fixed output
  eNR_SET_ASNIN_ID,					// Assign input
  eNR_SET_ASNOUT_ID,					// Assign output
  eNR_SET_AIN_ID,						// Analog input
  eNR_SET_AOUT_ID,					// Analog output
  eNR_SET_DIN_ID,						// Digital input
  eNR_SET_DOUT_ID,					// Digital output
  eNR_SET_FSNS_ID,					// Force sensor
  eNR_SET_FCTRL_SFT_ID,				// Force control shift
  eNR_SET_PRG_ID,						// Program number
  eNR_SET_STP_ID,						// Step number
  eNR_SET_TRQ_ID,						// torque
  eNR_SET_OUTTRQ_ID,					// output torque
  eNR_SET_PLAYBACK_ID,				// Playback
} NR_SET_PACK_DATA_ID;

//------------------------------------------------
// Set OutputDebugLog Mode
//------------------------------------------------
typedef	enum NR_LogExportMode
{
  LOG_EXPORT_MODE_OFF = 0,
  LOG_EXPORT_MODE_ON,
} NR_LOGEXPORTMODE;


//================================================
// STRUCTURE
//================================================

//------------------------------------------------
// Open() I/F
//------------------------------------------------

typedef	struct nachi_CommIfInfo
{
  char	*pcAddrs;				//! Ip Address
  long	lPortNo;				//! Port No
  long	lRetry;					//! Send Retry
  long	lSendTimeOut;			//! Send TimeOut msec( Data Recv --> Correct Data Send TimeOut )
  long	lCommSide;				//! Communication Side( NR_OBJECT_INTERNAL/NR_OBJECT_EXTERNAL_TCP/NR_OBJECT_EXTERNAL_UDP)
  long	lMode;					//! Internal:0 only, External:Server(NR_MODE_COMM_SERVER)/Client(NR_MODE_COMM_CLIENT)
  long	lKind;					//! Internal:Real(0)/XML(1), External:0 only
}	NACHI_COMMIF_INFO;

#pragma pack( push, 1 )

//------------------------------------------------
// I/F GetAll
//------------------------------------------------

// DEVISE-->APP

typedef struct NR_GetCtrlInfo{
  unsigned short		ushEstopBit 	: 1; 		// Estop
  unsigned short		ushPlaybkBit 	: 1; 		// Playbk
  unsigned short		ushConnectBit 	: 1; 		// Connect
  unsigned short		ushErrorBit 	: 5; 		// Error
  unsigned short		ushMotorBit   	: 1;		// Motor
  unsigned short		ushRsv        	: 6;		// Rsv
  unsigned short		ushProtcolBit 	: 1;		// Protcol
} NR_GET_CTRL_INFO;

typedef struct NR_GetRealDataBodyExt{
  float fCurTcpPos[ NR_MAX_AXIS ]; 			// CurTcpPos
  float fComTcpPos[ NR_MAX_AXIS ]; 			// ComTcpPos
  float fWorkCoord[ NR_MAX_XYZRPY ]; 			// WorkCoord
  float fToolCoord[ NR_MAX_XYZRPY ]; 			// ToolCoord
  float fCurAngle[ NR_MAX_AXIS ]; 			// fCurAngle
  float fComAngle[ NR_MAX_AXIS ]; 			// ComAngle
  float fTorque[ NR_MAX_AXIS ]; 				// Torque
  float fCurrent[ NR_MAX_AXIS ]; 				// Current
  bool bDigOut[ NR_MAX_DIGITAL_SIG ]; 		// DigOut
  bool bDigIn[ NR_MAX_DIGITAL_SIG ]; 			// DigIn
  float fAnaOut[ NR_MAX_ANALOG_CH ]; 			// AnaOut
  float fAnaIn[ NR_MAX_ANALOG_CH ]; 			// AnaIn
} NR_GET_REAL_DATA_BODY_EXT;

typedef struct NR_GetRealDataBodyStd{
  float fCurTcpPos[ NR_MAX_AXIS_STD ]; 		// CurTcpPos
  float fCurAngle[ NR_MAX_AXIS_STD ]; 		// CurAngle
  float fCurrent[ NR_MAX_AXIS_STD ]; 			// Current
  bool bDigOut[ NR_MAX_DIGITAL_SIG ]; 		// DigOut
  bool bDigIn[ NR_MAX_DIGITAL_SIG ]; 			// DigIn
} NR_GET_REAL_DATA_BODY_STD;

typedef struct NR_GetRealDataAll{
  NR_GET_CTRL_INFO stCtrl;					// Ctrl
  int nTime; 									// Time
  union {
    NR_GET_REAL_DATA_BODY_EXT	stExt;		// EXT(ushProtcolBit=1)
    NR_GET_REAL_DATA_BODY_STD	stStd;		// STD(ushProtcolBit=0)
  } ustData;
} NR_GET_REAL_DATA_ALL;

//------------------------------------------------
// I/F SetAll
//------------------------------------------------
// APP-->DEVISE


typedef struct NR_SetCtrlInfo{
  unsigned short		ushEstopBit 	: 1; 		// Estop
  unsigned short		ushFinishBit 	: 1; 		// Finish
  unsigned short		ushOrderBit 	: 1; 		// Order
  unsigned short		ushRsv        	: 12;		// Rsv
  unsigned short		ushProtcolBit 	:  1;		// Protcol
} NR_SET_CTRL_INFO;

typedef struct NR_SetRealDataBodyExt{
  float fComTcpPos[ NR_MAX_AXIS ]; 			// ComTcpPos
  float fWorkCoord[ NR_MAX_XYZRPY ]; 			// WorkCoord
  float fToolCoord[ NR_MAX_XYZRPY ]; 			// ToolCoord
  float fComAngle[ NR_MAX_AXIS ];		 		// ComAngle
  bool bDigOut[ NR_MAX_DIGITAL_SIG ]; 		// DigOut
  float fAnaOut[ NR_MAX_ANALOG_CH ]; 			// AnaOut
} NR_SET_REAL_DATA_BODY_EXT;

typedef struct NR_SetRealDataBodyStd{
  float fComTcpPos[ NR_MAX_AXIS_STD ]; 		// ComTcpPos
  float fWorkCoord[ NR_MAX_XYZRPY ]; 			// WorkCoord
  float fToolCoord[ NR_MAX_XYZRPY ]; 			// ToolCoord
  float fComAngle[ NR_MAX_AXIS_STD ];			// ComAngle
  bool bDigOut[ NR_MAX_DIGITAL_SIG ]; 		// DigOut
} NR_SET_REAL_DATA_BODY_STD;

typedef struct NR_SetRealDataAll{
  NR_SET_CTRL_INFO stCtrl;					// tCtrl
  int nTime;									// Time
  union {
    NR_SET_REAL_DATA_BODY_EXT	stExt;		// EXT(ushProtcolBit=1)
    NR_SET_REAL_DATA_BODY_STD	stStd;		// STD(ushProtcolBit=0)
  } ustData;
} NR_SET_REAL_DATA_ALL;

typedef struct NR_Notification {
  int		nErrCode;
  int		nUnitNo;
  int		nMechNo;
  int		nAxisNo;
  char*	csProg;
  int		nStepNo;
} NR_NOTIFICATION;

typedef struct _SYSTEMTIME {
  unsigned short		wYear;
  unsigned short		wMonth;
  unsigned short		wDayOfWeek;
  unsigned short		wDay;
  unsigned short		wHour;
  unsigned short		wMinute;
  unsigned short		wSecond;
  unsigned short		wMilliseconds;
} SYSTEMTIME;

typedef struct NR_Shift {
  float				fX;
  float				fY;
  float				fZ;
  float				fRoll;
  float				fPitch;
  float				fYaw;
} NR_SHIFT;

typedef struct NR_Pose	{
  float				fX;
  float				fY;
  float				fZ;
  float				fRoll;
  float				fPitch;
  float				fYaw;
} NR_POSE;

typedef struct NR_PalletReg {
  int					RegID;
  int					Execute;
  int					Kind;
  int					Layer;
  int					Work;
} NR_PALLETREG;

typedef struct NR_PalletWork {
  float				PalletID;
  float				Length;
  float				Width;
  float				Height;
  float				dL;
  float				dW;
  float				Xa;
  float				Ya;
} NR_PALLETWORK;

typedef struct NR_PalletLayer {
  float				LayerNum;
  float				TotalHeight;
  float				LayerType;
  struct {
    float			PleneID;
    float			Height;
  } Layer[50];
} NR_PALLETLAYER;

typedef struct NR_PalletPlene {
  float				Type;
  float				WorkNum[2];
  float				Shift[2];
  struct {
    float			PosX;			// X
    float			PosY;			// Y
    float			PosZ;			// Z
    float			ThetaZ;			// ??z
    float			Approach;
  } Work[99];
} NR_PALLETPLENE;

typedef struct NR_Pack {
  unsigned long	dwDtNum;
  struct {
    unsigned long	dwCounter;
    struct {
      union {
        float	fDt;
        unsigned long	dwDt;
        int				nDt;
      };
    } Data[64];
  } Que[40];
} NR_PACK;

typedef struct NR_Scope {
  struct {
    float			fTime;
    float			fData[6];
  } Buffer[40];
} NR_SCOPE;

typedef struct NR_FtpEnable {
  int				nEnable;
  const char*		csPassword;
} NR_FTP_ENABLE;

typedef struct NR_FtpStatus {
  int				nAnonymous;
  int				nConnectNum;
  int				nTimeout;
  const char*		csHomeDir;
  int				nDirectoryState;
  const char*		csLoginMessage;
  const char*		csPassword;
} NR_FTP_STATUS;

#pragma pack( pop )

// Notify Method Callback Method Pointer
typedef void(*Notify)(NR_Notification*);

/*!	@brief		open communication
 *	@param[in]	NACHI_COMMIF_INFO *pInfo	
 *	@param[out]	none	
 *	@retval		SUCCESS		connect object id
 *				FAIL		Error Code
 */
NR_EXTENDED_DLL_API int NR_Open( NACHI_COMMIF_INFO *pInfo );

/*!	@brief		close communication 
 *	@param[in]	nOpenId			connect object id
 *								0(All Close) 
 *	@retval		SUCCESS			NR_E_NORMAL
 *				FAIL			Error Code
 */
NR_EXTENDED_DLL_API int NR_Close(int nOpenId );

/*!	@brief		Ctrl Moter ON/OFF
 *	@param[in]	nOpenId			connect object id
 *	@param[in]	lCtrlSW			ON(1)/OFF(0)
 *	@retval		over Zero		Normal
 *				under Zero		Error
 */
NR_EXTENDED_DLL_API int NR_CtrlMotor(int nOpenId, long lCtrlSW);

/*!	@brief		Ctrl Run START/STOP
 *	@param[in]	nOpenId			connect object id
 *	@param[in]  lCtrlSW			START(1)/STOP(0)
 *	@param[in]	nUnitId			unit id
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlRun(int nOpenId, long lCtrlSW, int nUnitId = 1);

/*!	@brief		Real Data Get(All)
 *	@param[in]	nOpenId			connect object id
 *	@param[in]	NR_GET_REAL_DATA_ALL *pData	
 *	@param[in]	long lAccessMode(NR_ACCESS_WAIT/NR_ACCESS_NO_WAIT)
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_GetAll(int nOpenId, NR_GET_REAL_DATA_ALL *pData, long lAccessMode = NR_ACCESS_NO_WAIT);

/*!	@brief		Real Data Set(All)
 *	@param[in]	nOpenId			connect object id	
 *	@param[in]	NR_SET_REAL_DATA_ALL *pData	
 *	@param[in]	long lAccessMode(NR_ACCESS_WAIT/NR_ACCESS_NO_WAIT)
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_SetAll(int nOpenId, NR_SET_REAL_DATA_ALL *pData, long lAccessMode = NR_ACCESS_NO_WAIT);

/*!	@brief		Error notify
 *	@param[in]	nOpenId			connect object id	
 *	@param[in]	Notify			Callback method
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_SetNotify NR_SetNotifyW
#else
#define NR_SetNotify NR_SetNotifyA
#endif
NR_EXTENDED_DLL_API int NR_SetNotifyW(int nOpenId, Notify notify);
NR_EXTENDED_DLL_API int NR_SetNotifyA(int nOpenId, Notify notify);

/*!	@brief		Data Send(embedded mode)
 *	@param[in]	nOpenId			connect object id	
 *	@param[in]	char*			sendBuf
 *	@param[in]	int				nBufSize
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_Send(int nOpenId, char* sendBuf, int nBufSize);

/*!	@brief		Data Receive(embedded mode)
 *	@param[in]	nOpenId			connect object id	
 *	@param[in]	char*			recvBuf
 *	@param[in]	int				nBufSize
 *	@param[in]	int				nTimeOut
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_Recv(int nOpenId, char* recvBuf, int nBufSize, int nTimeOut);

/*!	@brief		Program / Pose File upload
 *	@param[in]	pszIpaddress	IP address	
 *	@param[in]	pstrRemoteFile	Remote file
 *	@param[in]	pstrLocalFile	Local File
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_UpLoad(const char* pszIpaddress, const char* pstrRemoteFile, const char* pstrLocalFile);


/*!	@brief		Program / Pose File download
 *	@param[in]	pszIpaddress	IP address	
 *	@param[in]	pstrRemoteFile	Remote file
 *	@param[in]	pstrLocalFile	Local File
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_DownLoad(const char* pszIpaddress, const char* pstrRemoteFile, const char* pstrLocalFile);

/*!	@brief		Move by Jog
 *	@param[in]	nOpenId			connect object id	
 *  @param[n]	pose			posture
 *	@param[in]	nUnitId			unit id
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlJog(int nOpenId, NR_POSE* pose, int nUnitId = 1);

/*!	@brief		Move by Absolute TCP Position
 *	@param[in]	nOpenId			connect object id	
 *  @param[in]	pose			posture
 *  @param[in]	nConf			config
 *	@param[in]	fExtPos			extra position
 *	@param[in]	nExtPosSize		extra position num
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlMoveX(int nOpenId, NR_POSE* pose, int nType = 0, int nUnitId = 1, int nConf = 0, float fExtPos[] = NULL, int nExtPosSize = 0);

/*!	@brief		Move by Relative Robot Position
 *	@param[in]	nOpenId			connect object id	
 *  @param[in]	pose			posture
 *  @param[in]	nType			0:Path / 1:Pause / 2:End
 *	@param[in]	nUnitId			unit id
 *  @param[in]	nConf			config
 *	@param[in]	fExtPos			extra position
 *	@param[in]	nExtPosSize		extra position num
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlMoveXR(int nOpenId, NR_POSE* pose, int nType = 0, int nUnitId = 1, int nConf = 0, float fExtPos[] = NULL, int nExtPosSize = 0);

/*!	@brief		Move by Relative Tool Position
 *	@param[in]	nOpenId			connect object id	
 *  @param[in]	pose			posture
 *  @param[in]	nType			0:Path / 1:Pause / 2:End
 *	@param[in]	nUnitId			unit id
 *  @param[in]	nConf			config
 *	@param[in]	fExtPos			extra position
 *	@param[in]	nExtPosSize		extra position num
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlMoveXT(int nOpenId, NR_POSE* pose, int nType = 0, int nUnitId = 1, int nConf = 0, float fExtPos[] = NULL, int nExtPosSize = 0);

/*!	@brief		Move by Absolute Axis Angle
 *	@param[in]	nOpenId			connect object id	
 *  @param[in]	fAngle			axis angle
 *  @param[in]	nAngleSize		axis size
 *  @param[in]	nType			0:Path / 1:Pause / 2:End
 *	@param[in]	nUnitId			unit id
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlMoveJ(int nOpenId, float fAngle[], int nAngleSize, int nType = 0, int nUnitId = 1);

/*!	@brief		Move by Relative Axis
 *	@param[in]	nOpenId			connect object id	
 *  @param[in]	fAngle			axis angle
 *  @param[in]	nAngleSize		axis size
 *  @param[in]	nType			0:Path / 1:Pause / 2:End
 *	@param[in]	nUnitId			unit id
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlMoveJA(int nOpenId, float fAngle[], int nAngleSize, int nType = 0, int nUnitId = 1);

/*!	@brief		Set Program No
 *	@param[in]	nOpenId			connect object id	
 *  @param[in]	nProgNo			program no
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlProgram(int nOpenId, int nProgNo);

/*!	@brief		Set Step No
 *	@param[in]	nOpenId			connect object id	
 *  @param[in]	nStepNo			step no
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlStep(int nOpenId, int nStepNo);

/*!	@brief	 		Control expand param
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlExpandParam(int nOpenId);

/*!	@brief	 		Get signal of MotorsOn.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOMotorsOn(int nOpenId, bool* value,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of GStop1.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOGStop1(int nOpenId, bool* value,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Start1.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOStart1(int nOpenId, bool* value,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Start2.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOStart2(int nOpenId, bool* value,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Start3.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOStart3(int nOpenId, bool* value,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Start4.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOStart4(int nOpenId, bool* value,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Stop.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOStop(int nOpenId, bool* value,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of PlayBack.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOPlayBack(int nOpenId, bool* value,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of MatSwitch.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOMatSwitch(int nOpenId, bool* value,
                                               int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of HighSpeed Teach.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOHighSpeedTeach(int nOpenId, bool* value,
                                                    int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of P1 Correct.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOP1Correct(int nOpenId, bool* value,
                                               int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of External Emergency Stop.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOExtStop(int nOpenId, bool* value,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Emergency Stop.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOEStop(int nOpenId, bool* value,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Safety Plug.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOSafetyPlug(int nOpenId, bool* value,
                                                int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Confirm Motors On/OFF.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOConfirmMotorsOn(int nOpenId, bool* value,
                                                     int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of TP Emergency Stop.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOTPEStop(int nOpenId, bool* value,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Teach Mode.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOTeach(int nOpenId, bool* value,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of TP Enable Switch.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOTPEnableSW(int nOpenId, bool* value,
                                                int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Cron.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOCRON(int nOpenId, bool* value,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Servo On.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOServoOn(int nOpenId, bool* value,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Servo Enable.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOServoEnable(int nOpenId, bool* value,
                                                 int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Magnet On.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOMagentON(int nOpenId, bool* value,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Weld Detection.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOWeldDetection(int nOpenId, bool* value,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of InConsistency.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOInConsistency(int nOpenId, bool* value,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Fixed Input Signal.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOInputSignal(int  nOpenId, bool value[], int nSubId, int nCount,
                                                 int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Motors On Lamp.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOMotorsOnLAMP(int nOpenId, bool* value,
                                                  int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Motors On Request.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOMotorsOnRequest(int nOpenId, bool* value,
                                                     int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Start Display1.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOStartDisplay1(int nOpenId, bool* value,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Start Display2.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOStartDisplay2(int nOpenId, bool* value,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Start Display3.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOStartDisplay3(int nOpenId, bool* value,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Start Display4.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOStartDisplay4(int nOpenId, bool* value,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Stop Display.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOStopDisplay(int nOpenId, bool* value,
                                                 int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of TP Enable Release.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOTPEnableRelease(int nOpenId, bool* value,
                                                     int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Motors On Enable.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOMotorsOnEnable(int nOpenId, bool* value,
                                                    int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Magnet On Enable.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOMagnetOnEnable(int nOpenId, bool* value,
                                                    int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of Internal/External Select.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOInternalExternal(int nOpenId, bool* value,
                                                      int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of WPS Emergency Stop Control.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOWPSEStop(int nOpenId, bool* value,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of CPU Failure.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOCPUFailure(int nOpenId, bool* value,
                                                int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of TP Mode.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOTPMode(int nOpenId, bool* value,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get signal of External Motors On.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOEXTMotorsOn(int nOpenId, bool* value,
                                                 int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Fixed Output Signal.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFixedIOOutputSignal(int nOpenId, bool value[], int nSubId, int nCount,
                                                  int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief			Get General Input Signal.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsGeneralInputSignal(int nOpenId, bool value[], int nSubId, int nCount,
                                                 int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get General Input Signal(Binary).
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			8 bool data to 1 int data
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsGeneralInputSignalB(int nOpenId, int value[], int nSubId, int nCount,
                                                  int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set General Output Signal.
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			true:on, false:off
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsGeneralOutputSignal(int nOpenId, bool value[], bool bUpdate, int nSubId, int nCount,
                                                  int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set General Output Signal(Binary).
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			8 bool data to 1 int data
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsGeneralOutputSignalB(int nOpenId, int value[], bool bUpdate, int nSubId, int nCount,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Input Signal Name.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			signal name
 *	@param[in]		nBufSize		buffer size of each string value	
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsStrInputSignalName NR_AcsStrInputSignalNameW
#else
#define NR_AcsStrInputSignalName NR_AcsStrInputSignalNameA
#endif
NR_EXTENDED_DLL_API int NR_AcsStrInputSignalNameW(int nOpenId, wchar_t* value[], size_t nBufSize, int nSubId, int nCount,
                                                  int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsStrInputSignalNameA(int nOpenId, char* value[], size_t nBufSize, int nSubId, int nCount,
                                                  int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Output Signal Name.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			signal name
 *	@param[in]		nBufSize		buffer size of each string value	
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsStrOutputSignalName NR_AcsStrOutputSignalNameW
#else
#define NR_AcsStrOutputSignalName NR_AcsStrOutputSignalNameA
#endif
NR_EXTENDED_DLL_API int NR_AcsStrOutputSignalNameW(int nOpenId, wchar_t* value[], size_t nBufSize, int nSubId, int nCount,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsStrOutputSignalNameA(int nOpenId, char* value[], size_t nBufSize, int nSubId, int nCount,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Global Integer Variable.
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			global integer variable
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsGlobalInt(int nOpenId, int value[], bool bUpdate, int nSubId, int nCount,
                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Global Real Variable.
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			global real variable
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsGlobalFloat(int nOpenId, float value[], bool bUpdate, int nSubId, int nCount,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Global String Variable.
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			global string variable
 *	@param[in]		szBufSize		buffer size of each string value
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsGlobalString NR_AcsGlobalStringW
#else
#define NR_AcsGlobalString NR_AcsGlobalStringA
#endif
NR_EXTENDED_DLL_API int NR_AcsGlobalStringW(int nOpenId, wchar_t* value[], size_t szBufSize, bool bUpdate, int nSubId, int nCount,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsGlobalStringA(int nOpenId, char* value[], size_t szBufSize, bool bUpdate, int nSubId, int nCount,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Local Integer Variable.
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			local integer data
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsLocalInt(int nOpenId, int value[], bool bUpdate, int nSubId, int nCount, int nUnitId = 1,
                                       int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Local Real Variable.
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			local real data
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsLocalFloat(int nOpenId, float value[], bool bUpdate, int nSubId, int nCount, int nUnitId = 1,
                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Local String Variable.
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			local string data
 *	@param[in]		szBufSize		buffer size of each string value
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsLocalString NR_AcsLocalStringW
#else
#define NR_AcsLocalString NR_AcsLocalStringA
#endif
NR_EXTENDED_DLL_API int NR_AcsLocalStringW(int nOpenId, wchar_t* value[], size_t szBufSize, bool bUpdate, int nSubId, int nCount, int nUnitId = 1,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsLocalStringA(int nOpenId, char* value[], size_t szBufSize, bool bUpdate, int nSubId, int nCount, int nUnitId = 1,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Shift Variable.
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			shift value
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsShift(int nOpenId, NR_SHIFT value[], bool bUpdate, int nSubId, int nCount,
                                    int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Version.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			version string
 *	@param[in]		szBufSize		buffer size of string value
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsVersion NR_AcsVersionW
#else
#define NR_AcsVersion NR_AcsVersionA
#endif
NR_EXTENDED_DLL_API int NR_AcsVersionW(int nOpenId, wchar_t* value, size_t szBufSize);
NR_EXTENDED_DLL_API int NR_AcsVersionA(int nOpenId, char* value, size_t szBufSize);

/*!	@brief	 		Get UnitName.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			unit name string
 *	@param[in]		szBufSize		buffer size of each string value
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsUnitName NR_AcsUnitNameW
#else
#define NR_AcsUnitName NR_AcsUnitNameA
#endif
NR_EXTENDED_DLL_API int NR_AcsUnitNameW(int nOpenId, wchar_t* value[], size_t szBufSize, int nSubId, int nCount);
NR_EXTENDED_DLL_API int NR_AcsUnitNameA(int nOpenId, char* value[], size_t szBufSize, int nSubId, int nCount);

/*!	@brief	 		Get Unit Axis Count.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Axis Count
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsUnitAxisCnt(int nOpenId, int value[], int nSubId, int nCount);

/*!	@brief	 		Get/Set Current Unit No.
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			current unit no
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsCurrentUnitNo(int nOpenId, int* value, bool bUpdate,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Current Mechanism No.
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			current mechanism no
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsCurrentMechaNo(int nOpenId, int* value, bool bUpdate,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Remote Mode.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			remote mode
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsRemoteMode(int nOpenId, bool* value,
                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program No.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			program no
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsPrgNo(int  nOpenId, int* value, int nCount, int nUnitId = 1,
                                    int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Step No.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			step no
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsStepNo(int nOpenId, int value[], int nCount, int nUnitId = 1,
                                     int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get User Level.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			user level
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsUserLevel(int nOpenId, int* value, bool bUpdate,
                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Err Info.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			error info
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsErrInfo(int nOpenId, int value[], int nSubId, int nCount,
                                      int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Axis Encode.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis encode
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsAxisEncode(int nOpenId, int value[], int nSubId, int nCount,
                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Axis Ampere Value.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis ampere value
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsAxisAmpValue(int nOpenId, float value[], int nSubId, int nCount,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Axis Order Ampere Value.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis order ampere value
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsAxisOrderAmpValue(int  nOpenId, float value[], int nSubId, int nCount,
                                                int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Axis Speed.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis speed
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsAxisSpeed(int nOpenId, float value[], int nSubId, int nCount,
                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Axis Order Speed.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis speed
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsAxisOrderSpeed(int nOpenId, float value[], int nSubId, int nCount,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Axis Theta.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis theta
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsAxisTheta(int nOpenId, float value[], int nSubId, int nCount,
                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Axis Theta Order.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis theta order
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsAxisThetaOrder(int nOpenId, float value[], int nSubId, int nCount,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Axis Torque.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis torque
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsAxisTorque(int nOpenId, float value[], int nSubId, int nCount,
                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Axis Tool Tip Position.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis tool tip position
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsToolTipPos(int nOpenId, float value[], int nSubId, int nCount,
                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Axis Order Tool Tip Position.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis order tool tip position
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsOrderToolTipPos(int nOpenId, float value[], int nSubId, int nCount,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Deflect Correction.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			deflect correction
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsDeflectCorrection(int nOpenId, float value[], int nSubId, int nCount,
                                                int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Order Tcp Speed.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			order tcp speed
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsOrderTcpSpeed(int nOpenId, float value[], int nSubId, int nCount,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Tcp Speed.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			tcp speed
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsTcpSpeed(int nOpenId, float value[], int nSubId, int nCount,
                                       int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Proportion Tcp Speed Analog.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			propotion tcp speed analog
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsProportionTcpSpeedAnalog(int nOpenId, float value[], int nSubId, int nCount,
                                                       int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Proportion Tcp Speed Digital.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			propotion tcp speed digital
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsProportionTcpSpeedDigital(int nOpenId, int value[], int nSubId, int nCount,
                                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Torque Imbalance.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			torque imbalance
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsTorqueImbalance(int nOpenId, float value[], int nSubId, int nCount,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Torque Axis Max.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			torque axis max
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsTorqueAxisMax(int nOpenId, int value[], int nSubId, int nCount,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Current Online Shift.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			current online shift
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsShiftOnlineCurrent(int nOpenId, NR_SHIFT value[], int nUnitId = 1,
                                                 int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Current Robot Shift.
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			current robot shift
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsShiftRobotCurrent(int nOpenId, NR_SHIFT value[], bool bUpdate, int nUnitId = 1,
                                                int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Current all Shift.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			current all shift
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsShiftAllCurrent(int nOpenId, NR_SHIFT value[], int nUnitId = 1,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Servo Motor ON/OFF.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			servo motor on/off
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsServoMotorOnOff(int nOpenId, int value[], int nSubId, int nCount,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Electronics Power Cost.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			electronics power cost.
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsElePowerCost(int nOpenId, float* value,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Servo Motor Status.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			servo motor status
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsServoMotorStatus(int nOpenId, int value[], int nSubId, int nCount,
                                               int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Servo Motor Error Status.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			servo motor error status
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsServoMotorErrStatus(int nOpenId, int value[], int nSubId, int nCount,
                                                  int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Encorder error count (trans)
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Encorder error count
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsEncErrCntTrans(int nOpenId, int value[], int nSubId, int nCount,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Encorder error count (Bit jump)
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Encorder error count
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsEncErrCntBitJmp(int nOpenId, int value[], int nSubId, int nCount,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Encorder error count (status)
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Encorder error count
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsEncErrCntStatus(int nOpenId, int value[], int nSubId, int nCount,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Encorder error count (Any ID)
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Encorder error count
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsEncErrCntAny(int nOpenId, int value[], int nSubId, int nCount,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Servo communication error cnt
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			 Servo communication error
 *	@param[in]		bPush			true:get by push, false:get by pull
 *	@param[in]		nUnitId			unit id
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsServoCommErr(int nOpenId, int* value,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get CPU load
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			CPU load
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsCPULoad(int nOpenId, float* value,
                                      int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get CPU voltage 3V
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			CPU voltage
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsCPUVoltage3(int nOpenId, float* value,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get CPU voltage 5V
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			CPU voltage
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsCPUVoltage5(int nOpenId, float* value,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get CPU temperature sensor1
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			CPU  temperature sensor1
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsCPUTempe1(int nOpenId, float* value,
                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get CPU temperature sensor2
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			CPU  temperature sensor1
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsCPUTempe2(int nOpenId, float* value,
                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Gas pressure base torque
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Gas pressure base torque
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsGasPrsBase(int nOpenId, float value[], int nSubId, int nCount,
                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Gas pressure Measure torque
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Gas pressure Measure torque
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsGasPrsMeasure(int nOpenId, float value[], int nSubId, int nCount,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Robot temperature
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Robot temperature
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsRobotTempe(int nOpenId, float value[], int nSubId, int nCount,
                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Overhaul
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Overhaul
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsOverHaul(int nOpenId, float value[], int nSubId, int nCount,
                                       int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Life span
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Life span
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsLifeSpan(int nOpenId, float value[], int nSubId, int nCount,
                                       int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program start time
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Program start time
 *	@param[in]		szBufSize		buffer size of string value
 *	@param[in]		nSubId			program no
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsProgStartTime NR_AcsProgStartTimeW
#else
#define NR_AcsProgStartTime NR_AcsProgStartTimeA
#endif
NR_EXTENDED_DLL_API int NR_AcsProgStartTimeW(int nOpenId, wchar_t* value, size_t szBufSize, int nSubId, int nUnitId = 1,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsProgStartTimeA(int nOpenId, char* value, size_t szBufSize, int nSubId, int nUnitId = 1,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Progra cycle time
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Progra cycle time
 *	@param[in]		szBufSize		buffer size of string value
 *	@param[in]		nSubId			program no
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsProgCycleTime NR_AcsProgCycleTimeW
#else
#define NR_AcsProgCycleTime NR_AcsProgCycleTimeA
#endif
NR_EXTENDED_DLL_API int NR_AcsProgCycleTimeW(int nOpenId, wchar_t* value, size_t szBufSize, int nSubId, int nUnitId = 1,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsProgCycleTimeA(int nOpenId, char* value, size_t szBufSize, int nSubId, int nUnitId = 1,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program total cycle
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			Program total cycle
 *	@param[in]		nSubId			program no
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsProgCycleCnt(int nOpenId, int* value, int nSubId, int nUnitId = 1,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program I wait time
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Program I wait time
 *	@param[in]		szBufSize		buffer size of string value
 *	@param[in]		nSubId			program no
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsProgIWaitTime NR_AcsProgIWaitTimeW
#else
#define NR_AcsProgIWaitTime NR_AcsProgIWaitTimeA
#endif
NR_EXTENDED_DLL_API int NR_AcsProgIWaitTimeW(int nOpenId, wchar_t* value, size_t szBufSize, int nSubId, int nUnitId = 1,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsProgIWaitTimeA(int nOpenId, char* value, size_t szBufSize, int nSubId, int nUnitId = 1,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program Delay wait time
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Program Delay wait time
 *	@param[in]		szBufSize		buffer size of string value
 *	@param[in]		nSubId			program no
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsProgDelayTime NR_AcsProgDelayTimeW
#else
#define NR_AcsProgDelayTime NR_AcsProgDelayTimeA
#endif
NR_EXTENDED_DLL_API int NR_AcsProgDelayTimeW(int nOpenId, wchar_t* value, size_t szBufSize, int nSubId, int nUnitId = 1,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsProgDelayTimeA(int nOpenId, char* value, size_t szBufSize, int nSubId, int nUnitId = 1,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program weld time
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Program weld time
 *	@param[in]		szBufSize		buffer size of string value
 *	@param[in]		nSubId			program no
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsProgWeldTime NR_AcsProgWeldTimeW
#else
#define NR_AcsProgWeldTime NR_AcsProgWeldTimeA
#endif
NR_EXTENDED_DLL_API int NR_AcsProgWeldTimeW(int nOpenId, wchar_t* value, size_t szBufSize, int nSubId, int nCount = 1, int nUnitId = 1,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsProgWeldTimeA(int nOpenId, char* value, size_t szBufSize, int nSubId, int nCount = 1, int nUnitId = 1,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program weld count
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Program weld count
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsProgWeldCnt(int nOpenId, int value[], int nSubId, int nCount, int nUnitId = 1,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program average speed
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Program average speed
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsProgAveSpeed(int nOpenId, float value[], int nSubId, int nCount, int nUnitId = 1,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program average torque
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Program average torque
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsProgAveTorque(int nOpenId, float value[], int nSubId, int nCount, int nUnitId = 1,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program average Current
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Program average Current
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsProgAveCurrent(int nOpenId, float value[], int nSubId, int nCount, int nUnitId = 1,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program max speed
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Program max speed
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsProgMaxSpeed(int nOpenId, float value[], int nSubId, int nCount, int nUnitId = 1,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program max Torque
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Program max Torque
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsProgMaxTorque(int nOpenId, float value[], int nSubId, int nCount, int nUnitId = 1,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program max Current
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Program max Current
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsProgMaxCurrent(int nOpenId, float value[], int nSubId, int nCount, int nUnitId = 1,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program life span
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Program life span
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsProgLifeSpan(int nOpenId, float value[], int nSubId, int nCount, int nUnitId = 1,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Analog Input.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			analog input
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsInputAnalog(int nOpenId, float value[], int nSubId, int nCount,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Analog Output.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			analog output
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsOutputAnalog(int nOpenId, float value[], bool bUpdate, int nSubId, int nCount,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Digital Input.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			digital input
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsInputDigital(int nOpenId, int value[], int nSubId, int nCount,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Digital Output.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			digital output
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsOutputDigital(int nOpenId, int value[], int nSubId, int nCount,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Seam Weld No.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			seam weld no.
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsSeamWeldNo(int nOpenId, int value[], int nSubId, int nCount,
                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Seam Mechanism on Move Side.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			seam mechanism on move side.
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsSeamMechaNoMoveSide(int nOpenId, int value[], int nSubId, int nCount,
                                                  int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Seam Mechanism on Stationary Side.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			seam mechanism on stationary side.
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsSeamMechaNoStationarySide(int nOpenId, int value[], int nSubId, int nCount,
                                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Seam Rotation on Move Side.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			seam rotation on move side.
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsSeamRotationMoveSide(int nOpenId, float value[], int nSubId, int nCount,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Seam Rotation on Stationary Side.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			seam rotation on stationary side.
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsSeamRotationStationarySide(int nOpenId, float value[], int nSubId, int nCount,
                                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Seam Weld Distance.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			seam weld distance.
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsSeamWeldDistance(int nOpenId, float value[], int nSubId, int nCount,
                                               int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Seam Weld Time.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			seam weld time.
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsSeamWeldTime(int nOpenId, float value[], int nSubId, int nCount,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Seam Energization Distance.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			seam energization distance.
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsSeamEnergizationDistance(int  nOpenId, float value[], int nSubId, int nCount,
                                                       int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Seam Energization Time.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			seam energization time.
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsSeamEnergizationTime(int nOpenId, float value[], int nSubId, int nCount,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Pressure Order Servo Gun.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			pressure order servo gun
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsPressureOrderServoGun(int nOpenId, float value[], int nSubId, int nCount,
                                                    int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Pressure Real Servo Gun.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			pressure real servo gun
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsPressureRealServoGun(int nOpenId, float value[], int nSubId, int nCount,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Friction Servo Gun.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			friction servo gun
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFrictionServoGun(int nOpenId, float value[], int nSubId, int nCount,
                                               int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Friction Servo Gun on Move Side
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			friction servo gun on move side
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFrictionServoGunMoveSide(int nOpenId, float value[], int nSubId, int nCount,
                                                       int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Friction Servo Gun on Stationary Side
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			friction servo gun on stationary side
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFrictionServoGunStationarySide(int nOpenId, float value[], int nSubId, int nCount,
                                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Physics PLC InputSignal 
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			physics PLC inputsignal
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsInputSignalPhysicsPlc(int nOpenId, bool value[], int nSubId, int nCount,
                                                    int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Physics PLC OutputSignal 
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			physics PLC outputsignal
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsOutputSignalPhysicsPlc(int nOpenId, bool value[], int nSubId, int nCount,
                                                     int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set PLC Bool Value 
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			PLC bool value
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsBoolValuePlc(int nOpenId, bool value[], bool bUpdate, int nSubId, int nCount,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set PLC Real Value 
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			PLC real value
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsRealValuePlc(int nOpenId, float value[], bool bUpdate, int nSubId, int nCount,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set PLC Dint Value 
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			PLC dint value
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsDintValuePlc(int nOpenId, int value[], bool bUpdate, int nSubId, int nCount,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set PLC Sint Value 
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			PLC sint value
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsSintValuePlc(int nOpenId, int value[], bool bUpdate, int nSubId, int nCount,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set PLC Timer Value 
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			PLC timer value
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsTimerValuePlc(int nOpenId, int value[], bool bUpdate, int nSubId, int nCount,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set PLC String Value 
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			PLC string value
 *	@param[in]		szBufSize		buffer size of each string value
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsStringValuePlc NR_AcsStringValuePlcW
#else
#define NR_AcsStringValuePlc NR_AcsStringValuePlcA
#endif
NR_EXTENDED_DLL_API int NR_AcsStringValuePlcW(int nOpenId, wchar_t* value[], size_t szBufSize, bool bUpdate, int nSubId, int nCount,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsStringValuePlcA(int nOpenId, char* value[], size_t szBufSize, bool bUpdate, int nSubId, int nCount,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Palletizing String Value 
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			palletizing string value
 *	@param[in]		szBufSize		buffer size of each string value
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsPalletizingStringValue NR_AcsPalletizingStringValueW
#else
#define NR_AcsPalletizingStringValue NR_AcsPalletizingStringValueA
#endif
NR_EXTENDED_DLL_API int NR_AcsPalletizingStringValueW(int nOpenId, wchar_t* value[], size_t szBufSize, bool bUpdate, int nSubId, int nCount,
                                                      int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsPalletizingStringValueA(int nOpenId, char* value[], size_t szBufSize, bool bUpdate, int nSubId, int nCount,
                                                      int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Palletizing Counter 
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			palletizing counter
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			index of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsPalletizingCounter(int nOpenId, int* value, bool bUpdate, int nSubId,
                                                 int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Palletizing Register
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			palletizing register
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			index of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL	
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsPalletizingReg(int nOpenId, NR_PALLETREG* value, bool bUpdate, int nSubId,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Palletizing Work
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			palletizing work
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			index of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsPalletizingWork(int nOpenId, NR_PALLETWORK* value, bool bUpdate, int nSubId,
                                              int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Palletizing Layer
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			palletizing layer
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			index of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsPalletizingLayer(int nOpenId, NR_PALLETLAYER* value, bool bUpdate, int nSubId,
                                               int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Palletizing Plene
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			palletizing plene
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			index of data
 *	@param[in]		nExtId			index of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsPalletizingPlene(int nOpenId, NR_PALLETPLENE* value, bool bUpdate, int nSubId, int nExtId,
                                               int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Record Speed
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			record speed
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsRecordSpeed(int nOpenId, int* value, bool bUpdate, int nUnitId = 1,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsRecordSpeedF(int nOpenId, float* value, bool bUpdate, int nUnitId = 1,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsRecordSpeedV(int nOpenId, float* value, bool bUpdate, int nUnitId = 1,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Record Tool No
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			record tool no
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsRecordToolNo(int nOpenId, int* value, bool bUpdate, int nUnitId = 1,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Record Accuracy No
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			record tool no
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsRecordAccuracyNo(int nOpenId, int* value, bool bUpdate, int nUnitId = 1,
                                               int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Record Smooth No
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			record smooth no
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsRecordSmoothNo(int nOpenId, int* value, bool bUpdate, int nUnitId = 1,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Acceleration No
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			acceleration no
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsAccelerationNo(int nOpenId, int* value, bool bUpdate, int nUnitId = 1,
                                             int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Interpolation Kind
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			interpolation no
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsInterpolationKind(int nOpenId, int* value, bool bUpdate, int nUnitId = 1,
                                                int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Posture Value
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			posture value
 *	@param[in]		bUpdate			TRUE:set, FALSE:get
 *	@param[in]		nId				value id
 *  @param[in]		nSubId			value subid
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsPoseValue(int nOpenId, NR_POSE* value, bool bUpdate, int nId, int nSubId, int nUnitId = 1,
                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Program Transfer ON/OFF
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			program transfer ON/OFF
 	@param[in]		nSubId			program no.
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsPrgTransferOnOff(int nOpenId, int* value, int nSubId, int nUnitId = 1,
                                               int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Entry Manual Coordinate Type
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			entry manual coordinate type
 *	@param[in]		szBufSize		buffer size of each string value
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsEntryManualCoordinateType NR_AcsEntryManualCoordinateTypeW
#else
#define NR_AcsEntryManualCoordinateType NR_AcsEntryManualCoordinateTypeA
#endif
NR_EXTENDED_DLL_API int NR_AcsEntryManualCoordinateTypeW(int nOpenId, wchar_t* value, size_t szBufSize,
                                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsEntryManualCoordinateTypeA(int OpenId, char* value, size_t szBufSize,
                                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Manual Coordinate Type
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			manual coordinate type
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsManualCoordinateType(int nOpenId, int* value, bool bUpdate,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Manual User Coordinate Type
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			manual coordinate type
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsManualUserCoordNo(int nOpenId, int* value, bool bUpdate,
                                                int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Manual Speed
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			manual speed
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsManualSpeed(int nOpenId, int* value, bool bUpdate,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Check Speed
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			check speed
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsCheckSpeed(int nOpenId, int* value, bool bUpdate,
                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Check Mode
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			check mode
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsCheckMode(int nOpenId, bool* value, bool bUpdate,
                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Check Operation Progress
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			check operation orogress
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsCheckOperationProgress(int nOpenId, float* value, int nUnitId = 1,
                                                     int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Playback Operation Mode
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			playback operation mode
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsOperationModePlayback(int nOpenId, int* value, bool bUpdate,
                                                    int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Waitting StatusSignal
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			waitting status signal
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsWaittingStatusSignal(int nOpenId, bool* value, bool bUpdate, int nUnitId = 1,
                                                   int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Prgram Stack Depth
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			prgram stack depth
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsStackDepthPrg(int nOpenId, int* value, int nUnitId = 1,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Over Ride Speed
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			over ride speed
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsOverRideSpeed(int nOpenId, float* value, int nUnitId = 1,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Over Ride Speed
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			waitting status signal
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsSpeedOverride(int nOpenId, float* value, bool bUpdate, int nUnitId = 1,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get/Set Machine lock
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			waitting status signal
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nUnitId			unit id
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsMachineLock(int nOpenId, int* value, bool bUpdate, int nUnitId = 1,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Slow Playback Status
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			slow playback status
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsStatusSlowPlayback(int nOpenId, int* value,
                                                 int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Saving Energy Status
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			saving energy status
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsStatusSavingEnergy(int nOpenId, int* value,
                                                 int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Force Cntrol
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			force cntrol
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsForceCtrl(int nOpenId, float value[], int nSubId, int nCount,
                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Force Shift
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			force shift
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsForceShift(int nOpenId, float value[], int nSubId, int nCount,
                                         int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Conveyer Register
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			conveyer register
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsConveyerRegister(int nOpenId, float value[], int nSubId, int nCount,
                                               int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Conveyer Mode
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			conveyer mode
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsConveyerMode(int nOpenId, int* value,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Conveyer Speed
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			conveyer speed
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsConveyerSpeed(int nOpenId, float value[], int nSubId, int nCount,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Conveyer Pulse
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			conveyer pulse
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsConveyerPulse(int nOpenId, int* value, int nSubId, int nCount,
                                            int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief			Get General Input Signal.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			true:on, false:off
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsGeneralInputSignalOnDesk(int nOpenId, bool value[], bool bUpdate, int nSubId, int nCount,
                                                       int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get General Input Signal(Binary).
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			8 bool data to 1 int data
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsGeneralInputSignalBOnDesk(int nOpenId, int value[], bool bUpdate, int nSubId, int nCount,
                                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Motor trip
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis encode
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsMotorTrip(int nOpenId, float value[], int nSubId, int nCount,
                                        int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Arm drop
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis encode
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsArmDrop(int nOpenId, float value[], int nSubId, int nCount,
                                      int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Interpolation Cycle
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis encode
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsIntpCycle(int nOpenId, int value[]);

/*!	@brief	 		Mechanism axis count
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis encode
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsMechanismAxisCnt(int nOpenId, int value[], int nSubId, int nCount = 1);

/*!	@brief	 		Mechanism count (BIT)
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis encode
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsMechanismCnt(int nOpenId, int value[]);

/*!	@brief	 		Unit count (BIT)
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis encode
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsUnitCnt(int nOpenId, int value[]);

/*!	@brief	 		Unit Mechanism count
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis encode
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsUnitMechanismCnt(int nOpenId, int value[], int nSubId, int nCount = 1);

/*!	@brief	 		Mechanism Name.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			signal name
 *	@param[in]		nBufSize		buffer size of each string value	
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsMechanismName NR_AcsMechanismNameW
#else
#define NR_AcsMechanismName NR_AcsMechanismNameA
#endif
NR_EXTENDED_DLL_API int NR_AcsMechanismNameW(int nOpenId, wchar_t* value[], size_t nBufSize, int nSubId, int nCount = 1);
NR_EXTENDED_DLL_API int NR_AcsMechanismNameA(int nOpenId, char* value[], size_t nBufSize, int nSubId, int nCount = 1);

/*!	@brief	 		Program / Pose Protect
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Protect
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			index of data
 *	@param[in]		nExtId			index of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsFileProtect(int nOpenId, int value[], bool bUpdate, int nSubId, int nExtId,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Run time
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			CPU load
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsRunTime(int nOpenId, float* value,
                                      int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Axis Run time.
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis ampere value
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsAxisRunTime(int nOpenId, float value[], int nSubId, int nCount,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Axis Motor RPD
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			axis ampere value
 *	@param[in]		nSubId			first index of data
 *	@param[in]		nCount			number of data
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsAxisMotorRPD(int nOpenId, float value[], int nSubId, int nCount,
                                           int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief		Set time
 *	@param[in]	nOpenId			connect object id	
 *  @param[in]	stTime			
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlTime(int nOpenId, SYSTEMTIME stTime);

/*!	@brief		Delete file
 *	@param[in]	nOpenId			connect object id
 *  @param[in]	lpFileName
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlDeleteFile(int nOpenId, const char* lpFileName);

/*!	@brief		Shot down
 *	@param[in]	nOpenId			connect object id
 *  @param[in]	bReboot
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlShutDown(int nOpenId, bool bReboot);

/*!	@brief		Restore BackUp
 *	@param[in]	nOpenId			connect object id
 *  @param[in]	lpBackupFolder
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlRestoreBackUp(int nOpenId, const char* lpBackupFolder);

/*!	@brief	 		Get/Set Encoder offset
 *	@param[in]		nOpenId			connect object id
 *	@param[in/out]	value			current unit no
 *	@param[in]		bUpdate			true:set, false:get
 *	@param[in]		nSubId			mechanism no
 *	@param[in]		nCount			number of data
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsEncOffsetValue(int nOpenId, int* value, bool bUpdate, int nSubId, int nCount);

/*!	@brief		Ctrl Event Log
 *	@param[in]	nOpenId			connect object id
 *	@param[in]  lOutputDir		INTERNAL(0)/EXTERNAL(1)
*	@param[in]	nEventLogType	SYSTEM(0)/APPLICATION(1)/SECURITY(2)
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlEventLog(int nOpenId, long lOutputDir = NR_INTERNAL_REQ, int nEventLogType = NR_EVENT_LOG_SYSTEM);

/*!	@brief		Ctrl Make Directory
 *	@param[in]	nOpenId			connect object id
 *	@param[in]  lOutputDir		NR_INTERNAL_REQ(0)/NR_RC_EXTERNAL_REQ(1)
 *	@param[in]  lpFolderName	The directory path to be created.
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_CtrlMakeDirectory NR_CtrlMakeDirectoryW
#else
#define NR_CtrlMakeDirectory NR_CtrlMakeDirectoryA
#endif
NR_EXTENDED_DLL_API int NR_CtrlMakeDirectoryW(int nOpenId, long lOutputDir, const wchar_t* lpDirectoryPath);
NR_EXTENDED_DLL_API int NR_CtrlMakeDirectoryA(int nOpenId, long lOutputDir, const char* lpDirectoryPath);

/*!	@brief		Macro START / STOP
 *	@param[in]	nOpenId			connect object id
 *  @param[in]	nMacroNo		Macro No.
 *  @param[in]	lCtrlSW			0: STOP / 1: START
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_CtrlMacroRun(int nOpenId, int nMacroNo, long lCtrlSW);

/*!	@brief	 		Get Macro Status(bit sum of executed macro thread)
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			the bit sum of executed macro thread
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsMacroStatus(int  nOpenId, int* value, int nMacroNo,
                                          int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Disk Space(Available)
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Disk space(available) C, D
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsDiskSpaceAvail(int nOpenId, float value[2], int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get Disk Space(Total)
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Disk space(Total) C, D
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsDiskSpaceTotal(int nOpenId, float value[2], int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		Get virtual memory value
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			Commit Charge(Avail) / Commit Charge(Total) / NRA2011.exe used
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
NR_EXTENDED_DLL_API int NR_AcsMemoryValue(int nOpenId, float value[3], int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		FTP service start / stop
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			NR_FTP_ENABLE( nEnable / csPassword )
 *	@param[in]		bUpdate			TRUE:set, FALSE:get
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsFtpEnable NR_AcsFtpEnableW
#else
#define NR_AcsFtpEnable NR_AcsFtpEnableA
#endif

NR_EXTENDED_DLL_API int NR_AcsFtpEnableW(int nOpenId, NR_FTP_ENABLE* value, bool bUpdate, int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsFtpEnableA(int nOpenId, NR_FTP_ENABLE* value, bool bUpdate, int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief	 		FTP service setting
 *	@param[in]		nOpenId			connect object id
 *	@param[out]		value			NR_FTP_STATUS(setting)
 *	@param[in]		bUpdate			TRUE:set, FALSE:get
 *	@param[in]		nPriority		a data update frequency when push transfer mode is used
 *	@param[in]		fThreshold		a tolerance area where the variable can change without sending a data update
 *	@retval			SUCCESS:NR_E_NORMAL
 *					FAIL:NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_AcsFtpStatus NR_AcsFtpStatusW
#else
#define NR_AcsFtpStatus NR_AcsFtpStatusA
#endif
NR_EXTENDED_DLL_API int NR_AcsFtpStatusW(int nOpenId, NR_FTP_STATUS* value, bool bUpdate, int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);
NR_EXTENDED_DLL_API int NR_AcsFtpStatusA(int nOpenId, NR_FTP_STATUS* value, bool bUpdate, int nPriority = NR_PULL_MODE, float fThreshold = 0.01f);

/*!	@brief		Program / Pose File upload(using user authentication)
 *	@param[in]	pszIpaddress	IP address
 *	@param[in]	pstrRemoteFile	Remote file
 *	@param[in]	pstrLocalFile	Local File
 *	@param[in]	pstrUser		User name@FTP Server
 *	@param[in]	pstrPassword	Password@FTP Server
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_UpLoadUser NR_UpLoadUserW
#else
#define NR_UpLoadUser NR_UpLoadUserA
#endif

NR_EXTENDED_DLL_API int NR_UpLoadUserW(const wchar_t* pszIpaddress, const wchar_t* pstrRemoteFile, const wchar_t* pstrLocalFile, const wchar_t* pstrUser, const wchar_t* pstrPassword);

NR_EXTENDED_DLL_API int NR_UpLoadUserA(const char* pszIpaddress, const char* pstrRemoteFile, const char* pstrLocalFile, const char* pstrUser, const char* pstrPassword);

/*!	@brief		Program / Pose File download(using user authentication)
 *	@param[in]	pszIpaddress	IP address
 *	@param[in]	pstrRemoteFile	Remote file
 *	@param[in]	pstrLocalFile	Local File
 *	@param[in]	pstrUser		User name@FTP Server
 *	@param[in]	pstrPassword	Password@FTP Server
 *	@retval		NR_E_NORMAL/NR_E_XXXXX
 */
#ifdef _UNICODE
#define NR_DownLoadUser NR_DownLoadUserW
#else
#define NR_DownLoadUser NR_DownLoadUserA
#endif

NR_EXTENDED_DLL_API int NR_DownLoadUserW(const wchar_t* pszIpaddress, const wchar_t* pstrRemoteFile, const wchar_t* pstrLocalFile, const wchar_t* pstrUser, const wchar_t* pstrPassword);

NR_EXTENDED_DLL_API int NR_DownLoadUserA(const char* pszIpaddress, const char* pstrRemoteFile, const char* pstrLocalFile, const char* pstrUser, const char* pstrPassword);

//<?????[?X????? ????????>
/*!	@brief		???O?t?@?C????o?????
 *	@param[in]	mode			???O?o???ON/OFF???
 *	@param[in]	pcFilePath		?o?????e?L?X?g?t?@?C????i?[??
 *	@param[in]	pcFileName		?o?????e?L?X?g?t?@?C????
 *	@retval		????:NR_E_NORMAL
 *				???:NR_E_XXXXX?i????l?j
 */
NR_EXTENDED_DLL_API int NR_ExportLog(NR_LOGEXPORTMODE mode, const char* pcFilePath, const char* pcFileName);
//<?????[?X????? ???????>
/*************************************************************************************************************************/
/*************************************************************************************************************************/
/*************************************************************************************************************************/
/*************************************************************************************************************************/
/*************************************************************************************************************************/
