/*
 * Copyright (C) 2012 Realtek Semiconductor Corp.
 * All Rights Reserved.
 *
 * This program is the proprietary software of Realtek Semiconductor
 * Corporation and/or its licensors, and only be used, duplicated,
 * modified or distributed under the authorized license from Realtek.
 *
 * ANY USE OF THE SOFTWARE OTHER THAN AS AUTHORIZED UNDER
 * THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * $Revision: 46475 $
 * $Date: 2014-02-14 11:03:12 +0800 (?±ä?, 14 äºŒæ? 2014) $
 *
 * Purpose : Define the CTC related extended OAM
 *
 * Feature : Provide CTC related extended OAM parsing and handling
 *
 */

/*
 * Include Files
 */
/* Standard include */
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <pthread.h>
#include <errno.h>
#include <semaphore.h>
#include <sys/wait.h>
#include <time.h>
/* EPON OAM include */
#include "epon_oam_config.h"
#include "epon_oam_err.h"
#include "epon_oam_db.h"
#include "epon_oam_dbg.h"
#include "epon_oam_rx.h"
#include "epon_oam_msgq.h"
/* User specific include */
#include "ctc_oam.h"
#include "ctc_oam_var.h"
/* Inlcude the variable table */
#include "ctc_oam_cli.h"
#include "ctc_oam_vartbl.c"
/* include CTC wrapper */
#include "ctc_wrapper.h"
#include "ctc_oam_alarmtbl.h"

#include "rtk/pon_led.h"
#include "rtk/gpio.h"
#include <rtk/epon.h>

/*
 * Symbol Definition
 */
#define CTC_OAM_MAC_AUTH_SET_COMMAND_NUM  3 /* FIXME: consider mac auth successfully after received 3 set commands */
/*
 * Data Declaration
 */
const ctc_infoOamVer_t supportedVerList[] = 
{
/*  {
 *      { OUI - 3 octets },
 *      version - 1 octets
 *  },
 */
    /* Pretend to support these versions */
#if CTC_OAM_SUPPORT_VERSION_01
    {
        { 0x11, 0x11, 0x11 },   /* CTC */
        0x01,                   /* Ver 0.1 */
    },
#endif
#if CTC_OAM_SUPPORT_VERSION_13
    {
        { 0x11, 0x11, 0x11 },   /* CTC */
        0x13                    /* Ver 1.3 */
    },
#endif
#if CTC_OAM_SUPPORT_VERSION_20
    {
        { 0x11, 0x11, 0x11 },   /* CTC */
        0x20                    /* Ver 2.0 */
    },
#endif
#if CTC_OAM_SUPPORT_VERSION_21
    {
        { 0x11, 0x11, 0x11 },   /* CTC */
        0x21                    /* Ver 2.1 */
    },
#endif
#if CTC_OAM_SUPPORT_VERSION_30
    {
        { 0x11, 0x11, 0x11 },   /* CTC */
        0x30                    /* Ver 3.0 */
    }
#endif
};
ctc_infoOamVer_t currCtcVer[EPON_OAM_SUPPORT_LLID_NUM];
static ctc_onuAuthLoid_t loidDb[EPON_OAM_SUPPORT_LLID_NUM];
static unsigned char ctc_onuAuth_state[EPON_OAM_SUPPORT_LLID_NUM];
static unsigned char ctc_onuAuth_failType[EPON_OAM_SUPPORT_LLID_NUM];
static unsigned char ctc_discovery_state[EPON_OAM_SUPPORT_LLID_NUM];
static ctc_swDlFile_t ctc_swdl_file; /* Single buffer for all LLID */
static pthread_t swDownload = 0; /* Download thread control */
static pthread_t swReboot = 0; /* Reboot thread control */
static pthread_t pmThread = 0; /* performance monitor thread control */
static pthread_cond_t pmThreadCond;
static pthread_mutex_t pmThreadMutex;
static ctc_churning_t ctc_last_churningKey[EPON_OAM_SUPPORT_LLID_NUM];
static sem_t ctcDataSem;
ctc_swDlBootInfo_t bootInfo;
ctc_onuSnInfo_t ctcOnuSn;
extern ctc_eventAlarmCode_t   *alarmDb;
extern int alarmNumber;
static int isCtcInit = 0;
static struct timeval dowload_start;
static unsigned int gpio_disTx_pin;

static unsigned char authType[EPON_OAM_SUPPORT_LLID_NUM]; /* 0: mac auth, 1: loid auth */
static unsigned int setCommandNum[EPON_OAM_SUPPORT_LLID_NUM];
static struct timeval epon_auth_succ_start[EPON_OAM_SUPPORT_LLID_NUM]; 

/*
 * Macro Definition
 */
#define CTC_SWDL_STATE_SET(state)   \
{                                   \
    sem_wait(&ctcDataSem);          \
    ctc_swdl_file.flag = state;     \
    sem_post(&ctcDataSem);          \
}

#define CTC_SWDL_STATE_GET()        (ctc_swdl_file.flag)

#define CTC_OAM_SWDOWNLOAD_MAX_FAKE_TIME 	60 /* MAX fake download time to avoid olt firmware upgrade timeout */

/*
 * Function Declaration
 */

static void dump_ctc_infoOam(
    ctc_infoOam_t *pInfoOam)
{
    ctc_infoOamVerRec_t *pItem;

    if(NULL != pInfoOam)
    {
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_DUMP,
            "CTC Info OAM:\n");
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_DUMP,
            "Type      : 0x%02x\n", pInfoOam->type);
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_DUMP,
            "Length    : %u\n", pInfoOam->length);
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_DUMP,
            "OUI       : %02x:%02x:%02x\n", pInfoOam->oui[0], pInfoOam->oui[1], pInfoOam->oui[2]);
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_DUMP,
            "ExtSupport: 0x%02x\n", pInfoOam->extSupport);
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_DUMP,
            "Version   : 0x%02x\n", pInfoOam->version);

        pItem = pInfoOam->vertionList;
        while(NULL != pItem)
        {
            EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_DUMP,
                "   OUI    : %02x:%02x:%02x\n", pItem->oui[0], pItem->oui[1], pItem->oui[2]);
            EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_DUMP,
                "   Version: 0x%02x\n", pItem->version);
            pItem = pItem->next;
        }
    }
}

int ctc_oam_onuAuthSuccTime_set(unsigned char llidIdx)
{
	gettimeofday(&epon_auth_succ_start[llidIdx], NULL);
	return EPON_OAM_ERR_OK;
}

/* auth fail or link down : return 0 
   auth success: return duration time after auth success */
int ctc_oam_onuAuthSuccTime_get(unsigned char llidIdx, long * authSuccTime)
{
	long uptime = 0;
	if(ctc_onuAuth_state[llidIdx] == CTC_OAM_ONUAUTH_STATE_SUCC)
	{
		struct timeval current;
		gettimeofday(&current, NULL);
		uptime = current.tv_sec - epon_auth_succ_start[llidIdx].tv_sec;
	}
	*authSuccTime = uptime;
	
	return EPON_OAM_ERR_OK;
}

int ctc_oam_onuAuthType_set(unsigned char llidIdx, unsigned char type)
{
	authType[llidIdx] = type;
	setCommandNum[llidIdx] = 0; /* reset set command count */ 

	return EPON_OAM_ERR_OK;
}

int ctc_oam_onuAuthType_get(unsigned char llidIdx,  unsigned char *type)
{
	*type = authType[llidIdx];

	return EPON_OAM_ERR_OK;
}

int ctc_oam_onuAuthLoid_set(unsigned char llidIdx, ctc_onuAuthLoid_t *pLoidAuth)
{
    /* Parameter check */
    if(llidIdx >= EPON_OAM_SUPPORT_LLID_NUM)
    {
        return EPON_OAM_ERR_PARAM;
    }
    if(NULL == pLoidAuth)
    {
        return EPON_OAM_ERR_PARAM;
    }

    loidDb[llidIdx] = *pLoidAuth;

    return EPON_OAM_ERR_OK;
}

int ctc_oam_onuAuthLoid_get(unsigned char llidIdx, ctc_onuAuthLoid_t *pLoidAuth)
{
    /* Parameter check */
    if(llidIdx >= EPON_OAM_SUPPORT_LLID_NUM)
    {
        return EPON_OAM_ERR_PARAM;
    }
    if(NULL == pLoidAuth)
    {
        return EPON_OAM_ERR_PARAM;
    }

    *pLoidAuth = loidDb[llidIdx];

    return EPON_OAM_ERR_OK;
}

int ctc_oam_onuAuthState_set(
    unsigned char llidIdx,
    unsigned char state,
    unsigned char failType)
{
    /* Parameter check */
    if(llidIdx >= EPON_OAM_SUPPORT_LLID_NUM)
    {
        return EPON_OAM_ERR_PARAM;
    }
    if(CTC_OAM_ONUAUTH_STATE_END <= state)
    {
        return EPON_OAM_ERR_PARAM;
    }

    if (CTC_OAM_ONUAUTH_STATE_FAIL == state)
    {
        rtk_pon_led_status_set(PON_LED_PON_MODE_EPON, PON_LED_STATE_AUTH_NG);
	#ifdef SUPPORT_OAM_EVENT_TO_KERNEL
		epon_oam_state_event_send(llidIdx, RTK_EPON_STATE_OAM_AUTH_FAIL);
	#endif
    }
    else if (CTC_OAM_ONUAUTH_STATE_SUCC == state)
    {
		epon_oam_register_success_number_increase(llidIdx);
	#ifdef SUPPORT_OAM_EVENT_TO_KERNEL
		epon_oam_state_event_send(llidIdx, RTK_EPON_STATE_OAM_AUTH_SUCC);
	#endif
		ctc_oam_onuAuthSuccTime_set(llidIdx);
		epon_oam_event_send(llidIdx, EPON_OAM_EVENT_AUTH_SUCCESS);
    }
    
    ctc_onuAuth_state[llidIdx] = state;
    ctc_onuAuth_failType[llidIdx] = failType;

	if((CTC_OAM_ONUAUTH_STATE_FAIL == state) || (CTC_OAM_ONUAUTH_STATE_SUCC == state))
		ctc_oam_onuAuthType_set(llidIdx, CTC_OAM_ONUAUTH_AUTH_LOID);
	
    return EPON_OAM_ERR_OK;
}

int ctc_oam_onuAuthState_get(
    unsigned char llidIdx,
    unsigned char *pState,
    unsigned char *pFailType)
{
    /* Parameter check */
    if(llidIdx >= EPON_OAM_SUPPORT_LLID_NUM)
    {
        return EPON_OAM_ERR_PARAM;
    }
    if(NULL == pState)
    {
        return EPON_OAM_ERR_PARAM;
    }

    *pState = ctc_onuAuth_state[llidIdx];
    if(NULL != pFailType)
    {
        *pFailType = ctc_onuAuth_failType[llidIdx];
    }

    return EPON_OAM_ERR_OK;
}

static void ctc_oam_verListItem_alloc(
    unsigned char oui[3],
    unsigned char ver,
    ctc_infoOamVerRec_t **ppVerListItem)
{
    ctc_infoOamVerRec_t *pItem;
    pItem = (ctc_infoOamVerRec_t *) malloc(sizeof(ctc_infoOamVerRec_t));

    if(NULL != pItem)
    {
        memcpy(pItem->oui, oui, 3);
        pItem->version = ver;
        pItem->next = NULL;
        *ppVerListItem = pItem;
    }
    else
    {
        *ppVerListItem = NULL;
    }
}

static void ctc_oam_verListItem_free(
    ctc_infoOamVerRec_t **ppVerListHead)
{
    ctc_infoOamVerRec_t *pVerList, *pVerListNext;

    pVerList = *ppVerListHead;
    while(NULL != pVerList)
    {
        pVerListNext = pVerList->next;
        free(pVerList);
        pVerList = pVerListNext;
    }
    *ppVerListHead = NULL;
}

static void ctc_oam_swDownloadBuf_clear(void)
{
    sem_wait(&ctcDataSem);
    if(ctc_swdl_file.fileName != NULL)
    {
        free(ctc_swdl_file.fileName);
        ctc_swdl_file.fileName = NULL;
    }

    if(ctc_swdl_file.mode != NULL)
    {
        free(ctc_swdl_file.mode);
        ctc_swdl_file.mode = NULL;
    }

    remove(CTC_ORGSPEC_SWDL_STORAGE);

    ctc_swdl_file.fileSize = 0;
    ctc_swdl_file.checkSize = 0;
    ctc_swdl_file.block = 0;
    /* Inside the semphore protection, don't use macro */
    ctc_swdl_file.flag = CTC_OAM_SWDOWNLOAD_BUF_CLEAR;
    sem_post(&ctcDataSem);

}

static void ctc_oam_swDownloadError_alloc(
    const ctc_swDownload_t inSwDownload, 
    ctc_swDownload_t *pOutSwDownload, 
    unsigned short errCode,
    char *pErrMsg)
{
    unsigned int strLen;

    pOutSwDownload->dataType = CTC_ORGSPEC_SWDL_TYPE_FILE;
    pOutSwDownload->opCode = CTC_ORGSPEC_SWDL_OPCODE_ERROR;
    pOutSwDownload->length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_ERROR_LEN;
    pOutSwDownload->tid = inSwDownload.tid;
    pOutSwDownload->parse.error.errCode = errCode;
    if(pErrMsg != NULL)
    {
        strLen = strlen(pErrMsg);
        pOutSwDownload->parse.error.errMsg = (char *)malloc(strLen);
        if(pOutSwDownload->parse.error.errMsg != NULL)
        {
            memcpy(pOutSwDownload->parse.error.errMsg, pErrMsg, strLen);
            pOutSwDownload->length += strLen;
        }
        else
        {
            /* Ignore the error message if allocate failed */
        }
    }
    else
    {
        pOutSwDownload->parse.error.errMsg = NULL;
    }
}

static void ctc_oam_swDownloadError_free(
    ctc_swDownload_t *pSwDownload) 
{
    if(pSwDownload->parse.error.errMsg != NULL)
    {
        free(pSwDownload->parse.error.errMsg);
    }
}

static int ctc_oam_swDownloadNv_set(
    char *varName,
    char *varValue)
{
    char execCommand[CTC_EXECMD_UV_CMD_LEN];
    int execCmdStatus;

    if ((varName == NULL) || (varValue == NULL))
    {
        return EPON_OAM_ERR_PARAM;
    }

    memset(execCommand, 0, CTC_EXECMD_UV_CMD_LEN);

    sprintf(execCommand, "/bin/nv setenv %s %s", varName, varValue);

    execCmdStatus = system(execCommand);
    if (-1 == execCmdStatus || 0 == WIFEXITED(execCmdStatus) || 0 != WEXITSTATUS(execCmdStatus))
    {
        return -1;
    }

    return 0;
}

static int ctc_oam_swDownloadNv_get(
    char *varName,
    char *varValue)
{
    char execCommand[CTC_EXECMD_UV_CMD_LEN];
    char stdoutBuffer[CTC_EXECMD_STDOUT_LEN];
    char *pStr, *pReturnStr = NULL;
    FILE *pFD;

    if ((varName == NULL) || (varValue == NULL))
    {
        return EPON_OAM_ERR_PARAM;
    }

    memset(execCommand, 0, CTC_EXECMD_UV_CMD_LEN);
    memset(stdoutBuffer, 0, CTC_EXECMD_STDOUT_LEN);

    sprintf(execCommand, "/bin/nv getenv %s", varName);

    pFD = popen(execCommand, "r");
    if(pFD != NULL)
    {
        if (fgets(stdoutBuffer, CTC_EXECMD_STDOUT_LEN, pFD))
        {
            pStr = strtok(stdoutBuffer, "=");

            if (pStr)
            {
                strncpy(varValue, stdoutBuffer + strlen(stdoutBuffer) + 1, CTC_EXECMD_UV_VALUE_LEN);
				pclose(pFD);
				return EPON_OAM_ERR_OK;
            }
        }
        pclose(pFD);
    }

    return EPON_OAM_ERR_NOT_FOUND;
}

static void *ctc_oam_swDlThread(void *argu)
{
    int ret;
    unsigned char writeTarget;
    char execCommand[64];

    /* Once started, chekc MD5 and program the flash 
     * It won't stop event if the EPON link down
     */
    /* Don't allow others to stop the download process */
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);

    /* Lock the data semaphore to prevent any change */
    sem_wait(&ctcDataSem);
    /* Execute the firmware upgrade script */
    writeTarget = (bootInfo.active == 0) ? 1 : 0;
    snprintf(execCommand, 64,
    	"/bin/sh -x /etc/scripts/fwu_starter.sh %d " CTC_ORGSPEC_SWDL_STORAGE, writeTarget);
    ret = system(execCommand);

    if(0 == ret)
    {
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
            "[OAM:%s:%d] Program to img #%d compelet\n", __FILE__, __LINE__, writeTarget);

        /* Inside the semphore protection, don't use macro */
        ctc_swdl_file.flag = CTC_OAM_SWDOWNLOAD_BUF_WRITEOK;
    }
    else
    {
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
            "[OAM:%s:%d] Program to img #%d failed\n", __FILE__, __LINE__, writeTarget);

        /* Inside the semphore protection, don't use macro */
        ctc_swdl_file.flag = CTC_OAM_SWDOWNLOAD_BUF_WRITEFAIL;
    }
    swDownload = 0;
    sem_post(&ctcDataSem);
    pthread_exit(NULL);

    return NULL;
}

static void *ctc_oam_rebootThread(void *argu)
{
    int ret;
    unsigned char writeTarget;
    char execCommand[CTC_EXECMD_UV_CMD_LEN];

    /* Wait 5 seconds to allow all oam to be replied */
    sleep(5);

	while(CTC_SWDL_STATE_GET() == CTC_OAM_SWDOWNLOAD_BUF_WRITEING)
	{	/* For nor flash: must wait fireware download to complete */
		EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO, 
			"[OAM:%s:%d] wait fireware download to complete\n", __FILE__, __LINE__);
		sleep(5);
	}
	
    /* Reboot the system */
    sprintf(execCommand, "reboot -f");
    ret = system(execCommand);

    if(0 == ret)
    {
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
            "[OAM:%s:%d] reboot command success\n", __FILE__, __LINE__);
    }
    else
    {
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
            "[OAM:%s:%d] reboot command fail\n", __FILE__, __LINE__);
    }
    pthread_exit(NULL);

    return NULL;
}

static void *ctc_oam_pmThread(void *argu)
{
    int ret;
    unsigned char run = 1;
    struct timeval now;
    struct timespec timeout;


    while (run) {
        /* Reset for CTC_OAM_PM_INTERVAL */
        pthread_mutex_lock(&pmThreadMutex);
        gettimeofday(&now, NULL);
        timeout.tv_sec = now.tv_sec + CTC_OAM_PM_INTERVAL;
        timeout.tv_nsec = now.tv_usec * 1000;
        ret = pthread_cond_timedwait(&pmThreadCond, &pmThreadMutex, &timeout);
        if(ret != ETIMEDOUT)
        {
            run = 0;
        }
        pthread_mutex_unlock(&pmThreadMutex);

        /* Call to the CTC wrapper for statistics update */
        CTC_WRAPPER(pmStatUpdate, CTC_OAM_PM_INTERVAL);

    }
    EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO, "[OAM:%s:%d] PM stopped\n", __FILE__, __LINE__);
    pmThread = 0;   
}

static void ctc_oam_Info_gen(
    unsigned short expectLen,
    unsigned char extSupport,
    unsigned char version,
    ctc_infoOam_t *pRemoteInfoOam,
    unsigned char genInfoList,
    unsigned char *pBuf)
{
    /* Buffer size check should be done before calling gen function */
    int i;
    unsigned char *pBufPtr = pBuf;

    pBufPtr[0] = EPON_INFO_OAMPDU_TYPE_ORG_SPEC;
    pBufPtr[1] = expectLen;
    pBufPtr[2] = pRemoteInfoOam->oui[0];
    pBufPtr[3] = pRemoteInfoOam->oui[1];
    pBufPtr[4] = pRemoteInfoOam->oui[2];
    pBufPtr[5] = extSupport;
    pBufPtr[6] = version;
    pBufPtr += CTC_INFO_OAM_MIN;
    if(0 != genInfoList)
    {
        for(i = 0;i < sizeof(supportedVerList)/sizeof(ctc_infoOamVer_t);i++)
        {
            pBufPtr[0] = supportedVerList[i].oui[0];
            pBufPtr[1] = supportedVerList[i].oui[1];
            pBufPtr[2] = supportedVerList[i].oui[2];
            pBufPtr[3] = supportedVerList[i].version;
            pBufPtr += CTC_INFO_OAM_VERITEM_LEN;
        }
    }
}

static void ctc_oam_orgSpecHdr_gen(
    unsigned char oui[],        /* OUI to be generate */
    unsigned char extOpcode,    /* Extended Opcode to be generate */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    /* Buffer size check should be done before calling gen function */

    pReplyBuf[0] = oui[0];
    pReplyBuf[1] = oui[1];
    pReplyBuf[2] = oui[2];
    pReplyBuf[3] = extOpcode;

    *pReplyLen = CTC_ORGSPEC_HDR_LEN;
}

static void ctc_oam_orgSpecVarInstant_gen(
    ctc_varInstant_t *pInstant, /* Instant data to be generate */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    /* Buffer size check should be done before calling gen function */
    unsigned char *pPtr;

    pPtr = pReplyBuf;
    *pPtr = pInstant->varDesc.varBranch;
    pPtr += 1; /* Branch */
#ifdef SUPPORT_OAM_BYTEORDER
	CTC_BUF_ENCODE16(pPtr, &pInstant->varDesc.varLeaf);
#else
    pPtr[0] = ((unsigned char *)(&pInstant->varDesc.varLeaf))[0];
    pPtr[1] = ((unsigned char *)(&pInstant->varDesc.varLeaf))[1];
#endif
    pPtr += 2; /* Leaf */
    *pPtr = pInstant->varWidth;
    pPtr += 1; /* Width */

    if(pInstant->varWidth == CTC_ORGSPEC_VARINSTANT_WIDTH20)
    {
        /*
         * CTC2.0 Index TLV:
         * ----------------
         * 1 	Branch
         * ----------------
         * 2 	Leaf
         * ----------------
         * 1 	Width=0x01
         * ----------------
         * 1	Value
         * ----------------			 
         */
        memcpy(pPtr, pInstant->varData, sizeof(unsigned char) * pInstant->varWidth);
    }
    else if(pInstant->varWidth == CTC_ORGSPEC_VARINSTANT_WIDTH21)
    {
        /*
		 * CTC2.1 Index TLV:
		 * ----------------
		 * 1 	Branch
		 * ----------------
		 * 2 	Leaf
		 * ----------------
		 * 1 	Width=0x04
		 * ----------------
		 * 4	Value	-------\
		 * ----------------     \
         *                      ----------
         *                      1 portType
         *                      ----------
         *                      1 chassisNo
         *                      ----------
         *                      1 slotNo
         *                      ----------
         *                      1 portNo
         *                      ----------
         */
        memcpy(pPtr, pInstant->varData, sizeof(unsigned char) * pInstant->varWidth);
    }

    pPtr += sizeof(unsigned char) * pInstant->varWidth; /* Data */

    *pReplyLen = pPtr - pReplyBuf;
}

static void ctc_oam_orgSpecVarContainer_gen(
    ctc_varContainer_t *pContainer, /* Instant data to be generate */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    /* Buffer size check should be done before calling gen function */
    unsigned char *pPtr;
    unsigned short remainLen;

    pPtr = pReplyBuf;
    *pPtr = pContainer->varDesc.varBranch;
    pPtr += 1; /* Branch */
#ifdef SUPPORT_OAM_BYTEORDER
	CTC_BUF_ENCODE16(pPtr, &pContainer->varDesc.varLeaf);
#else
    pPtr[0] = ((unsigned char *)(&pContainer->varDesc.varLeaf))[0];
    pPtr[1] = ((unsigned char *)(&pContainer->varDesc.varLeaf))[1];
#endif
    pPtr += 2; /* Leaf */

	/* Martin ZHU note:
	 * CTC_OAM_VAR_RESP_VARBADPARAM: pContainer->pVarData is NULL 
	 * CTC_OAM_VAR_RESP_VARNORESOURCE: pContainer->pVarData is NULL
	 * 2016-2-29
	 */
    if( (CTC_VAR_CONTAINER_INDICATOR & pContainer->varWidth)||(pContainer->varWidth==CTC_OAM_VAR_RESP_VARBADPARAM)||(pContainer->varWidth==CTC_OAM_VAR_RESP_VARNORESOURCE) )
    {
    	*pPtr = pContainer->varWidth & 0x00ff;
        pPtr += 1; /* Width */
        /* No data field */
    }
    else
    {
        remainLen = pContainer->varWidth;
        while(0 != remainLen)
        {
            if(remainLen >= 0x80)
            {
                *pPtr = 0x00;
                pPtr += 1; /* Width */
                memcpy(pPtr, pContainer->pVarData + pContainer->varWidth - remainLen, sizeof(unsigned char) * 0x80);
                pPtr += sizeof(unsigned char) * 0x80; /* Data */
                remainLen -= 0x80;

                if(remainLen > 0)
                {
                    /* Create consecutive variable container with the same branch/leaf */
                    *pPtr = pContainer->varDesc.varBranch;
                    pPtr += 1; /* Branch */
#ifdef SUPPORT_OAM_BYTEORDER
					CTC_BUF_ENCODE16(pPtr, &pContainer->varDesc.varLeaf);
#else
                    pPtr[0] = ((unsigned char *)(&pContainer->varDesc.varLeaf))[0];
                    pPtr[1] = ((unsigned char *)(&pContainer->varDesc.varLeaf))[1];
#endif
                    pPtr += 2; /* Leaf */
                }
            }
            else
            {
                *pPtr = remainLen;
                pPtr += 1; /* Width */
                memcpy(pPtr, pContainer->pVarData + pContainer->varWidth - remainLen, sizeof(unsigned char) * remainLen);
                pPtr += sizeof(unsigned char) * remainLen; /* Data */
                remainLen = 0;
            }
        }
    }

    *pReplyLen = pPtr - pReplyBuf;
}

static void ctc_oam_orgSpecOnuAuthLoid_gen(
    ctc_onuAuthLoid_t *pAuthLoid,  /* Auth LOID structure */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    /* Buffer size check should be done before calling gen function */
    unsigned char *pPtr;
    unsigned short dataLen;
    unsigned short length;

    pPtr = pReplyBuf;
    length = CTC_ORGSPEC_ONUAUTH_RESP_LOID_LEN;
    *pPtr = CTC_ORGSPEC_ONUAUTH_CODE_RESP;
    pPtr += 1; /* Auth code */
#ifdef SUPPORT_OAM_BYTEORDER
	CTC_BUF_ENCODE16(pPtr, &length);
#else
    pPtr[0] = ((unsigned char *) &length)[0];
    pPtr[1] = ((unsigned char *) &length)[1];
#endif
    pPtr += 2; /* Auth data length */
    *pPtr = CTC_ORGSPEC_ONUAUTH_LOID;
    pPtr += 1; /* Auth type */
    /* According to the CTC standard, NULL (0x00) should be padded before
     * the LOID/password
     */
    /* LOID */
    dataLen = strlen(pAuthLoid->loid);
    dataLen = (dataLen <= CTC_ORGSPEC_ONUAUTH_LOID_LEN) ? dataLen : CTC_ORGSPEC_ONUAUTH_LOID_LEN;
    memset(pPtr, 0, CTC_ORGSPEC_ONUAUTH_LOID_LEN);
    memcpy(pPtr + (CTC_ORGSPEC_ONUAUTH_LOID_LEN - dataLen), pAuthLoid->loid, dataLen);
    pPtr += CTC_ORGSPEC_ONUAUTH_LOID_LEN; /* LOID */
    /* Password */
    dataLen = strlen(pAuthLoid->password);
    dataLen = (dataLen <= CTC_ORGSPEC_ONUAUTH_PASSWORD_LEN) ? dataLen : CTC_ORGSPEC_ONUAUTH_PASSWORD_LEN;
    memset(pPtr, 0, CTC_ORGSPEC_ONUAUTH_PASSWORD_LEN);
    memcpy(pPtr + (CTC_ORGSPEC_ONUAUTH_PASSWORD_LEN - dataLen), pAuthLoid->password, dataLen);
    pPtr += CTC_ORGSPEC_ONUAUTH_PASSWORD_LEN; /* Password */

    *pReplyLen = pPtr - pReplyBuf;
}

static void ctc_oam_orgSpecOnuAuthNak_gen(
    unsigned char desireType,   /* Desire auth type */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    /* Buffer size check should be done before calling gen function */
    unsigned char *pPtr;
    unsigned short length;

    pPtr = pReplyBuf;
    length = CTC_ORGSPEC_ONUAUTH_RESP_NAK_LEN;
    *pPtr = CTC_ORGSPEC_ONUAUTH_CODE_RESP;
    pPtr += 1; /* Auth code */
#ifdef SUPPORT_OAM_BYTEORDER
	CTC_BUF_ENCODE16(pPtr, &length);
#else
    pPtr[0] = ((unsigned char *) &length)[0];
    pPtr[1] = ((unsigned char *) &length)[1];
#endif
    pPtr += 2; /* Auth data length */
    *pPtr = CTC_ORGSPEC_ONUAUTH_NAK;
    pPtr += 1; /* Auth type */
    *pPtr = desireType;
    pPtr += 1; /* Desire Auth type */

    *pReplyLen = pPtr - pReplyBuf;
}

static void ctc_oam_orgSpecSwDownload_gen(
    ctc_swDownload_t *pSwDl,    /* Software download data */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    /* Buffer size check should be done before calling gen function */
    unsigned char *pPtr;

    pPtr = pReplyBuf;
    *pPtr = pSwDl->dataType;
    pPtr += 1; /* dataType */
#ifdef SUPPORT_OAM_BYTEORDER
	CTC_BUF_ENCODE16(pPtr, &pSwDl->length);
#else
    pPtr[0] = ((unsigned char *) &pSwDl->length)[0];
    pPtr[1] = ((unsigned char *) &pSwDl->length)[1];
#endif
    pPtr += 2; /* software download data length */
#ifdef SUPPORT_OAM_BYTEORDER
	CTC_BUF_ENCODE16(pPtr, &pSwDl->tid);
#else
    pPtr[0] = ((unsigned char *) &pSwDl->tid)[0];
    pPtr[1] = ((unsigned char *) &pSwDl->tid)[1];
#endif
    pPtr += 2; /* TID */
#ifdef SUPPORT_OAM_BYTEORDER
	CTC_BUF_ENCODE16(pPtr, &pSwDl->opCode);
#else
    pPtr[0] = ((unsigned char *) &pSwDl->opCode)[0];
    pPtr[1] = ((unsigned char *) &pSwDl->opCode)[1];
#endif
    pPtr += 2; /* opCode */

    switch(pSwDl->opCode)
    {
    case CTC_ORGSPEC_SWDL_OPCODE_FILEREQ:
        /* TODO: not possible for ONU */
        break;
    case CTC_ORGSPEC_SWDL_OPCODE_FILEDATA:
        /* TODO: not possible for ONU */
        break;
    case CTC_ORGSPEC_SWDL_OPCODE_FILEACK:
#ifdef SUPPORT_OAM_BYTEORDER
		CTC_BUF_ENCODE16(pPtr, &pSwDl->parse.fileAck.block);
#else
        pPtr[0] = ((unsigned char *) &pSwDl->parse.fileAck.block)[0];
        pPtr[1] = ((unsigned char *) &pSwDl->parse.fileAck.block)[1];
#endif
        pPtr += 2; /* block */
        break;
    case CTC_ORGSPEC_SWDL_OPCODE_ERROR:
#ifdef SUPPORT_OAM_BYTEORDER
		CTC_BUF_ENCODE16(pPtr, &pSwDl->parse.error.errCode);
#else
        pPtr[0] = ((unsigned char *) &pSwDl->parse.error.errCode)[0];
        pPtr[1] = ((unsigned char *) &pSwDl->parse.error.errCode)[1];
#endif
        pPtr += 2; /* errCode */
        if(pSwDl->parse.error.errMsg != NULL)
        {
            memcpy(pPtr, pSwDl->parse.error.errMsg, strlen(pSwDl->parse.error.errMsg));
            pPtr += strlen(pSwDl->parse.error.errMsg);
        }
        *pPtr = 0;
        pPtr += 1; /* ASCII NULL */
        break;
    case CTC_ORGSPEC_SWDL_OPCODE_ENDREQ:
        /* TODO: not possible for ONU */
        break;
    case CTC_ORGSPEC_SWDL_OPCODE_ENDRESP:
        *pPtr = pSwDl->parse.endResp.respCode;
        pPtr += 1; /* respCode */
        break;
    case CTC_ORGSPEC_SWDL_OPCODE_ACTREQ:
        /* TODO: not possible for ONU */
        break;
    case CTC_ORGSPEC_SWDL_OPCODE_ACTRESP:
        *pPtr = pSwDl->parse.activateResp.ack;
        pPtr += 1; /* ack */
        break;
    case CTC_ORGSPEC_SWDL_OPCODE_COMMITREQ:
        /* TODO: not possible for ONU */
        break;
    case CTC_ORGSPEC_SWDL_OPCODE_COMMITRESP:
        *pPtr = pSwDl->parse.commitResp.ack;
        pPtr += 1; /* ack */
        break;
    default:
        break;
    }

    *pReplyLen = pPtr - pReplyBuf;
}

static void ctc_oam_orgSpecChurning_gen(
    ctc_churning_t *pChurning,  /* Churning key structure */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short *pReplyLen,  /* Reply size used by this handler*/
    unsigned int keyLen)  		/* churningKey length */
{
    /* Buffer size check should be done before calling gen function */
    unsigned char *pPtr;

    pPtr = pReplyBuf;
    *pPtr = pChurning->churningCode;
    pPtr += 1; /* churningCode */
    *pPtr = pChurning->keyIdx;
    pPtr += 1; /* keyIdx */
    if(CTC_ORGSPEC_CHURNING_NEWKEY == pChurning->churningCode)
    {
        memcpy(pPtr, pChurning->churningKey, keyLen);
        pPtr += keyLen; /* churningKey */
    }

    *pReplyLen = pPtr - pReplyBuf;
}

static int ctc_oam_orgSpecDba_calc(
    unsigned char dbaCode, /* DBA code to be encoded */
    ctc_dbaThreshold_t *pDbaThreshold)  /* DBA threshold structure */
{
    int i, j;
    unsigned short expectLen = 0;

    expectLen += CTC_ORGSPEC_DBA_HDR_LEN;
    switch(dbaCode)
    {
    case CTC_ORGSPEC_DBA_GET_REQ:
        /* Nothing to be encoded except DBA code */
        break;
    case CTC_ORGSPEC_DBA_SET_RESP:
        /* set_DBA_response has extra field Set ACK */
        expectLen += CTC_ORGSPEC_DBA_SETACK_LEN;
    case CTC_ORGSPEC_DBA_GET_RESP:
    case CTC_ORGSPEC_DBA_SET_REQ:
        expectLen += CTC_ORGSPEC_DBA_REPORTHDR_LEN;
        for(i = 0;i < pDbaThreshold->numQSet;i++)
        {
            expectLen += CTC_ORGSPEC_DBA_REPORTMAP_LEN;
            for(j = 0;j < CTC_ORGSPEC_DBA_QUEUE_MAX;j++)
            {
                if(pDbaThreshold->reportMap[i] & (1 << j))
                {
                    expectLen += CTC_ORGSPEC_DBA_QUEUETHRESHOLD_LEN;
                }
            }
        }
        break;
    }

    return expectLen;
}

static void ctc_oam_orgSpecDba_gen(
    unsigned char dbaCode, /* DBA code to be encoded */
    ctc_dbaThreshold_t *pDbaThreshold,  /* DBA threshold structure */
    unsigned char dbaSetAck,  /* DBA threshold set ACK,
                               * only valid if dbaCode = CTC_ORGSPEC_DBA_SET_RESP */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    /* Buffer size check should be done before calling gen function */
    int i, j;
    unsigned char *pPtr;

    pPtr = pReplyBuf;
    *pPtr = dbaCode;
    pPtr += 1; /* DBA code */

    switch(dbaCode)
    {
    case CTC_ORGSPEC_DBA_GET_REQ:
        /* Nothing to be encoded except DBA code */
        break;
    case CTC_ORGSPEC_DBA_SET_RESP:
        /* set_DBA_response has extra field Set ACK */
        *pPtr = dbaSetAck & 0x01;
        pPtr += 1; /* Set ACK */
    case CTC_ORGSPEC_DBA_GET_RESP:
    case CTC_ORGSPEC_DBA_SET_REQ:
        /* According to CTC spec, the number of queue set is actual queue set + 1*/
        *pPtr = pDbaThreshold->numQSet + 1;
        pPtr += 1; /* Number of Queue sets */
        for(i = 0;i < pDbaThreshold->numQSet;i++)
        {
            *pPtr = pDbaThreshold->reportMap[i];
            pPtr += 1; /* Report bitmap */
            for(j = 0;j < CTC_ORGSPEC_DBA_QUEUE_MAX;j++)
            {
                if(pDbaThreshold->reportMap[i] & (1 << j))
                {
#ifdef SUPPORT_OAM_BYTEORDER
					CTC_BUF_ENCODE16(pPtr, &pDbaThreshold->queueSet[i].queueThreshold[j]);
#else
                    pPtr[0] = ((unsigned char *)(&pDbaThreshold->queueSet[i].queueThreshold[j]))[0];
                    pPtr[1] = ((unsigned char *)(&pDbaThreshold->queueSet[i].queueThreshold[j]))[1];
#endif
                    pPtr += CTC_ORGSPEC_DBA_QUEUETHRESHOLD_LEN; /* Queue threshold */
                }
            }
        }
        break;
    }
    *pReplyLen = pPtr - pReplyBuf;
}

static unsigned char ctc_oam_version_support(
    unsigned char oui[3],
    unsigned char version)
{
    int i;
    for(i = 0;i < sizeof(supportedVerList)/sizeof(ctc_infoOamVer_t);i++)
    {
        if((supportedVerList[i].oui[0] == oui[0]) &&
           (supportedVerList[i].oui[1] == oui[1]) &&
           (supportedVerList[i].oui[2] == oui[2]) &&
           (supportedVerList[i].version == version))
        {
            return 1;
        }           
    }
    return 0;
}

int ctc_oam_orgSepcVarCb_get(
    ctc_varDescriptor_t varDesc,
    ctc_varCb_t *pVarCb)
{
    ctc_varCb_t *pVarCbDb;

    if(NULL == pVarCb)
    {
        return EPON_OAM_ERR_PARAM;
    }

    /* Search callback function according ot its branch */
    switch(varDesc.varBranch)
    {
    case CTC_VAR_REQBRANCH_STDATTR:
        pVarCbDb = ctc_stdAttrCb;
        break;
    case CTC_VAR_REQBRANCH_STDACT:
        pVarCbDb = ctc_stdActCb;
        break;
    case CTC_VAR_REQBRANCH_EXTATTR:
        pVarCbDb = ctc_extAttrCb;
        break;
    case CTC_VAR_REQBRANCH_EXTACT:
        pVarCbDb = ctc_extActCb;
        break;
    default:
        return EPON_OAM_ERR_NOT_FOUND;
    }

    while(0 != pVarCbDb->varDesc.varBranch)
    {
        if(pVarCbDb->varDesc.varLeaf == varDesc.varLeaf)
        {
            *pVarCb = *pVarCbDb;
            return EPON_OAM_ERR_OK;
        }
        /* Move to next array index */
        pVarCbDb = pVarCbDb + 1;
    }

    return EPON_OAM_ERR_NOT_FOUND;
}

static void ctc_oam_orgSepcVarNoWidth_get(
    ctc_varDescriptor_t varDesc,
    unsigned char *isNoWidth)
{
    ctc_varDescriptor_t *pVarDesc;

    pVarDesc = ctc_noWidthList;
    while(0 != pVarDesc->varBranch)
    {
        if((pVarDesc->varBranch == varDesc.varBranch) && (pVarDesc->varLeaf == varDesc.varLeaf))
        {
            *isNoWidth = 1;
            return;
        }
        /* Move to next array index */
        pVarDesc ++;
    }
    *isNoWidth = 0;

    return;
}

static int ctc_oam_orgSepcVarCascadeLen_get(
    ctc_varDescriptor_t varDesc,
    unsigned char *pBuf,
    unsigned short remainLen,
    unsigned short *length)
{
    unsigned short procLen = 0;
    unsigned short curWidth = 0;
    unsigned short totalLen = 0;

    while(remainLen >= CTC_ORGSPEC_VARCONTAINER_MIN)
    {
        if((*(pBuf+procLen) == varDesc.varBranch) && (*((unsigned short *)(pBuf+procLen+1)) == varDesc.varLeaf))
        {
            curWidth = (0 == *(pBuf+procLen+3)) ? 0x80 : *(pBuf+procLen+3);
            if(remainLen >= (curWidth + CTC_ORGSPEC_VARCONTAINER_MIN))
            {
                totalLen += curWidth;
                if(curWidth == 0x80)
                {
                    remainLen -= (curWidth + CTC_ORGSPEC_VARCONTAINER_MIN);
                    procLen += (curWidth + CTC_ORGSPEC_VARCONTAINER_MIN);
                }
                else
                {
                    break;
                }
            }
            else
            {
                /* Insufficient length for parsing */
                *length = 0;
                return EPON_OAM_ERR_PARSE;
            }
        }
        else
        {
            break;
        }
    }
    *length = totalLen;

    return EPON_OAM_ERR_OK;
}

static void ctc_oam_churningKey_rand(
    unsigned char key[], int length)
{
	int i;
	for(i = 0; i < length; i++)
    	key[i] = rand() % 0x100;
}

static int ctc_oam_orgSpecDbaTheshold_parse(
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned short *pParseLen,
    ctc_dbaThreshold_t *pDbaThreshold)
{
    int i, j;
    unsigned char *pPtr;
    unsigned short expectLen;

    *pParseLen = 0;
    if(length < CTC_ORGSPEC_DBA_REPORTHDR_LEN)
    {
        /* Insufficient length for parsing */
        return EPON_OAM_ERR_OK;
    }

    pPtr = pFrame;
    pDbaThreshold->numQSet = *pPtr;
    /* Due to CTC specified that the report's last queue set contains
     * the total amount of all queues, the number of queue set value
     * in the frame will always include that last queue set but no actual
     * queueu set in the frame
     * pDbaThreshold->numQSet = actual queue set number + 1
     */
    if((pDbaThreshold->numQSet < CTC_ORGSPEC_DBA_QUEUESET_MIN + 1) ||
       (pDbaThreshold->numQSet > CTC_ORGSPEC_DBA_QUEUESET_MAX + 1))
    {
        /* Incorrect value */
        return EPON_OAM_ERR_OK;
    }
    pDbaThreshold->numQSet -= 1; /* Change to actual queue set number */
    CTC_BUF_ADD(pPtr, length, CTC_ORGSPEC_DBA_REPORTHDR_LEN);

    for(i = 0;i < pDbaThreshold->numQSet;i++)
    {
        if(length < CTC_ORGSPEC_DBA_REPORTMAP_LEN)
        {
            /* Insufficient length for parsing */
            return EPON_OAM_ERR_OK;
        }
        pDbaThreshold->reportMap[i] = *pPtr;
        CTC_BUF_ADD(pPtr, length, CTC_ORGSPEC_DBA_REPORTMAP_LEN);

        /* Calculate the expect length from bitmap */
        expectLen = 0;
        for(j = 0;j < CTC_ORGSPEC_DBA_QUEUE_MAX;j++)
        {
            if(pDbaThreshold->reportMap[i] & (1 << j))
            {
                expectLen += CTC_ORGSPEC_DBA_QUEUETHRESHOLD_LEN;
            }
        }
        if(length < expectLen)
        {
            /* Insufficient length for parsing */
            return EPON_OAM_ERR_OK;
        }

		/* allow empty reportMap when olt want to clear dba (set all queue values to zero) */
		if(expectLen == 0)
		{
			for(j = 0;j < CTC_ORGSPEC_DBA_QUEUE_MAX;j++)
	        {
	        	pDbaThreshold->queueSet[i].queueThreshold[j] = 0;
			}
		}
		else
		{
	        for(j = 0;j < CTC_ORGSPEC_DBA_QUEUE_MAX;j++)
	        {
	            if(pDbaThreshold->reportMap[i] & (1 << j))
	            {
#ifdef SUPPORT_OAM_BYTEORDER
					CTC_BUF_PARSE16(pPtr, &pDbaThreshold->queueSet[i].queueThreshold[j]);
#else
	                ((unsigned char *)(&pDbaThreshold->queueSet[i].queueThreshold[j]))[0] = pPtr[0];
	                ((unsigned char *)(&pDbaThreshold->queueSet[i].queueThreshold[j]))[1] = pPtr[1];
#endif
	                CTC_BUF_ADD(pPtr, length, CTC_ORGSPEC_DBA_QUEUETHRESHOLD_LEN);
	            }
	        }
		}
    }
    *pParseLen = pPtr - pFrame;

    return EPON_OAM_ERR_OK;
}

static void ctc_oam_orgSpecEvent_gen(
    unsigned char  eventCode,
    ctc_eventOam_t *eventStatus,
    ctc_thresholdOam_t  *thresholdStatus,
    unsigned short count, 
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/        
{

    int ret = 0;
    unsigned char *pPtr;
    int i, offset= 0;
    ctc_eventAlarmCode_t * alarmEntry;
    
    pPtr = pReplyBuf;
    *pPtr = eventCode;
    pPtr += 1;
#ifdef SUPPORT_OAM_BYTEORDER
	CTC_BUF_ENCODE16(pPtr, &count);
#else
    pPtr[0] = ((unsigned char *) &count)[0];
    pPtr[1] = ((unsigned char *) &count)[1]; 
#endif
    pPtr += 2;

    switch(eventCode)
    {
        case CTC_ORGSPEC_EVENT_STATUS_REQ:
        case CTC_ORGSPEC_EVENT_STATUS_SET:
        case CTC_ORGSPEC_EVENT_STATUS_RESP:
            for(i = 0; i < count; i++ )
            {
#ifdef SUPPORT_OAM_BYTEORDER
				CTC_BUF_ENCODE16(&pPtr[offset], &eventStatus[i].objectType);
#else
                memcpy( &pPtr[offset], &(eventStatus[i].objectType),2);
#endif
                offset+=2;
#ifdef SUPPORT_OAM_BYTEORDER
				CTC_BUF_ENCODE32(&pPtr[offset], &eventStatus[i].objectInstance);
#else
                memcpy( &pPtr[offset], &(eventStatus[i].objectInstance),4);
#endif
                offset+=4;
#ifdef SUPPORT_OAM_BYTEORDER
				CTC_BUF_ENCODE16(&pPtr[offset], &eventStatus[i].alarmId);
#else
                memcpy( &pPtr[offset],  &(eventStatus[i].alarmId),2);
#endif
                offset+=2;
                
                alarmEntry = ctc_oam_alarm_get_entry(eventStatus[i].alarmId, eventStatus[i].objectInstance);    
                if(alarmEntry != NULL)
                {
#ifdef SUPPORT_OAM_BYTEORDER
					CTC_BUF_ENCODE32(&pPtr[offset], &alarmEntry->alarmEnable);
#else
                     memcpy( &pPtr[offset],&(alarmEntry->alarmEnable),4);                                         
#endif
                }
                else
                {               
#ifdef SUPPORT_OAM_BYTEORDER
					CTC_BUF_ENCODE32(&pPtr[offset], &eventStatus[i].eventStatus);
#else
                    memcpy( &pPtr[offset],&(eventStatus[i].eventStatus),4);
#endif
                }
                offset +=4;
            }
            pPtr += offset;
            break;
        case CTC_ORGSPEC_EVENT_THRESHOLD_REQ:
        case CTC_ORGSPEC_EVENT_THRESHOLD_SET:
        case CTC_ORGSPEC_EVENT_THRESHOLD_RESP:
            for(i = 0; i < count; i++ )
            {
#ifdef SUPPORT_OAM_BYTEORDER
				CTC_BUF_ENCODE16(&pPtr[offset], &thresholdStatus[i].objectType);
#else
            	memcpy(&pPtr[offset] ,&(thresholdStatus[i].objectType),2);
#endif
                offset += 2;
#ifdef SUPPORT_OAM_BYTEORDER
				CTC_BUF_ENCODE32(&pPtr[offset], &thresholdStatus[i].objectInstance);
#else
                memcpy(&pPtr[offset],&( thresholdStatus[i].objectInstance),4);
#endif
                offset += 4;
#ifdef SUPPORT_OAM_BYTEORDER
				CTC_BUF_ENCODE16(&pPtr[offset], &thresholdStatus[i].alarmId);
#else
                memcpy(&pPtr[offset],&( thresholdStatus[i].alarmId),2);
#endif
                offset += 2;
                
                alarmEntry = ctc_oam_alarm_get_entry(thresholdStatus[i].alarmId, thresholdStatus[i].objectInstance);    
                if(alarmEntry != NULL)
                {
#ifdef SUPPORT_OAM_BYTEORDER
					CTC_BUF_ENCODE32(&pPtr[offset], &alarmEntry->threshold);
#else
                    memcpy(&pPtr[offset],&(alarmEntry->threshold),4);
#endif
                    offset += 4;
#ifdef SUPPORT_OAM_BYTEORDER
					CTC_BUF_ENCODE32(&pPtr[offset], &alarmEntry->clearThreshold);
#else
                    memcpy(&pPtr[offset],&(alarmEntry->clearThreshold),4);
#endif
                    offset += 4;
                }
                else
                {
#ifdef SUPPORT_OAM_BYTEORDER
					CTC_BUF_ENCODE32(&pPtr[offset], &thresholdStatus[i].setThreshold);
#else
                    memcpy(&pPtr[offset],&(thresholdStatus[i].setThreshold),4);
#endif
                    offset += 4;
#ifdef SUPPORT_OAM_BYTEORDER
					CTC_BUF_ENCODE32(&pPtr[offset], &thresholdStatus[i].clearThreshold);
#else
                    memcpy(&pPtr[offset],&(thresholdStatus[i].clearThreshold),4);
#endif
                    offset += 4;
                }
            }
            pPtr += offset;
            break;
        default:
            break;
    }
    
    *pReplyLen = pPtr - pReplyBuf;
}

#if 0
#if CTC_OAM_SUPPORT_BENEATH_20
static int ctc_oam_orgSpecValidInstantLeaf_decode20(
    ctc_varInstant_t *pVarInstant)
{
    unsigned int value;

    switch(pVarInstant->varDesc.varLeaf)
    {
        case CTC_VAR_REQBRANCH_INSTANT_PORT:
            value = *pVarInstant->varData;
            /* Chassis & slot number are not defined in CTC 2.0- instant, always zero */
            pVarInstant->parse.uniPort.chassisNo = 0;
            pVarInstant->parse.uniPort.slotNo = 0;
            if(0 == value)
            {
                pVarInstant->parse.uniPort.portType = CTC_VAR_INSTANT_PORTTYPE_PON;
                pVarInstant->parse.uniPort.portNo = 0;
            }
            else if(value <= 0x4F)
            {
                pVarInstant->parse.uniPort.portType = CTC_VAR_INSTANT_PORTTYPE_ETHERNET;
                pVarInstant->parse.uniPort.portNo = value;
            }
            else if(value <= 0x8F)
            {
                pVarInstant->parse.uniPort.portType = CTC_VAR_INSTANT_PORTTYPE_VOIP;
                pVarInstant->parse.uniPort.portNo = value - 0x4F;
            }
            else if(value <= 0x9F)
            {
                pVarInstant->parse.uniPort.portType = CTC_VAR_INSTANT_PORTTYPE_E1;
                pVarInstant->parse.uniPort.portNo = value - 0x90;
            }
            else if(value < 0xFF)
            {
                pVarInstant->parse.uniPort.portType = CTC_VAR_INSTANT_PORTTYPE_OTHERS;
                pVarInstant->parse.uniPort.portNo = value - 0x9F;
            }
            else if(value == 0xFF)
            {
                pVarInstant->parse.uniPort.portType = CTC_VAR_INSTANT_PORTTYPE_ETHERNET;
                pVarInstant->parse.uniPort.portNo = CTC_VAR_INSTANT_PORTALL;
            }
            else
            {
                return 0;
            }
            /* Valid instant leaf value for CTC 2.0 */
            return 1;
        default:
            return 0;
    }
}
/* For CTC version smaller or equal to 2.0 */
static int ctc_oam_orgSpecVar_req20(
    unsigned char llidIdx,      /* LLID index of the incoming operation */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    int ret;
    unsigned char *pProcPtr, *pReplyPtr;
    unsigned short genLen;
    ctc_varCb_t varCb;
    ctc_varDescriptor_t varDesc;
    ctc_varInstant_t *pVarInstant = NULL, varInstant;
    ctc_varContainer_t varContainer;
    unsigned short remainLen;

    /* 1. Parse branch/leaf and search database for process function
     * 2. Call process function to preocess the branch/leaf
     */
    pProcPtr = pFrame;
    pReplyPtr = pReplyBuf;
    remainLen = length;
    *pReplyLen = 0;

    while(remainLen >= CTC_ORGSPEC_VARDESC_LEN)
    {
        varDesc.varBranch = *pProcPtr;
        memcpy(&varDesc.varLeaf, &pProcPtr[1], sizeof(unsigned short));
        CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_VARDESC_LEN); /* Variable Descriptor */

        switch(varDesc.varBranch)
        {
        case CTC_VAR_REQBRANCH_INSTANT20:
            /* Parse for variable instant */
            if((remainLen < CTC_ORGSPEC_VARINSTANT_LEN20) &&
               (bufLen > (CTC_ORGSPEC_VARDESC_LEN + CTC_ORGSPEC_VARINSTANT_LEN20)))
            {
                /* Ignore the following variables */
                *pReplyLen = (pReplyPtr - pReplyBuf);
                return EPON_OAM_ERR_OK;
            }
            varInstant.varDesc = varDesc;
            varInstant.varWidth = CTC_ORGSPEC_VARINSTANT_WIDTH20;
            CTC_BUF_ADD(pProcPtr, remainLen, 1); /* Width */
            memcpy(varInstant.varData, pProcPtr, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH20);
            CTC_BUF_ADD(pProcPtr, remainLen, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH20); /* Data */
            if(ctc_oam_orgSpecValidInstantLeaf_decode20(&varInstant))
            {
                /* Set instant pointer for all remain variable request */
                pVarInstant = &varInstant;
                if(!CTC_INSTANT_IS_ALLPORTS(pVarInstant))
                {
                    /* Encode the variable instant into reply packet */
                    ctc_oam_orgSpecVarInstant_gen(pVarInstant, pReplyPtr, &genLen);
                    CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
                }
                else
                {
                    /* For get request with ALL port instatnt, all port should response with its own data and instant 
                     * The response callback should encode the instant for each port
                     */
                }
            }
            else
            {
                /* Ignore the invalid variable instant */
                pVarInstant = NULL;
            }
            break;
        case CTC_VAR_REQBRANCH_STDATTR:
        case CTC_VAR_REQBRANCH_EXTATTR:
        case CTC_VAR_REQBRANCH_STDACT:
        case CTC_VAR_REQBRANCH_EXTACT:
            ret = ctc_oam_orgSepcVarCb_get(varDesc, &varCb);
            if((EPON_OAM_ERR_OK != ret) ||
               (NULL == varCb.handler_get) ||
               (0 == (varCb.op & CTC_VAR_OP_GET)))
            {
                /* Ignore unsupport branch/leaf */
                break;
            }

            /* Use branch/leaf specific callback function to process */
            varContainer.pVarData = NULL;
            ret = varCb.handler_get(llidIdx, CTC_VAR_OP_GET, pVarInstant, varDesc, &varContainer);
            if(EPON_OAM_ERR_OK == ret)
            {
                if(bufLen >= (CTC_ORGSPEC_VARDESC_LEN + varContainer.varWidth + 1 /* Width */))
                {
                    genLen = 0;
                    ctc_oam_orgSpecVarContainer_gen(
                        &varContainer,
                        pReplyPtr,
                        &genLen);
                    CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
                }
            }

            if(NULL != varContainer.pVarData)
            {
                free(varContainer.pVarData);
            }
            break;
        case CTC_VAR_REQBRANCH_END:
            *pReplyLen = (pReplyPtr - pReplyBuf);
            return EPON_OAM_ERR_OK;
        default:
            break;
        }
    }
    *pReplyLen = (pReplyPtr - pReplyBuf);

    return EPON_OAM_ERR_OK;
}

/* For CTC version smaller or equal to 2.0 */
static int ctc_oam_orgSpecSet_req20(
    unsigned char llidIdx,      /* LLID index of the incoming operation */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    int ret;
    unsigned int extFlag;
    unsigned char *pProcPtr, *pReplyPtr;
    unsigned short genLen;
    ctc_varCb_t varCb;
    ctc_varInstant_t *pVarInstant = NULL, varInstant;
    ctc_varContainer_t varContainer;
    unsigned short remainLen;

    /* 1. Parse branch/leaf and search database for process function
     * 2. Call process function to preocess the branch/leaf
     */
    pProcPtr = pFrame;
    pReplyPtr = pReplyBuf;
    remainLen = length;
    *pReplyLen = 0;

    while(remainLen >= CTC_ORGSPEC_VARCONTAINER_MIN)
    {
        varContainer.varDesc.varBranch = *pProcPtr;
        memcpy(&varContainer.varDesc.varLeaf, &pProcPtr[1], sizeof(unsigned short));
        varContainer.varWidth = pProcPtr[3];
        /* Mark the next line, when olt reboot onu, the varContainer have no the member or the varWidth = 0 */
        /* varContainer.varWidth = (0 == varContainer.varWidth) ? 0x80 : varContainer.varWidth; */
        CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_VARCONTAINER_MIN); /* Variable Container */

        switch(varContainer.varDesc.varBranch)
        {
        case CTC_VAR_REQBRANCH_INSTANT20:
            /* Parse for variable instant */
            if((remainLen < varContainer.varWidth) &&
               (bufLen > (CTC_ORGSPEC_VARDESC_LEN + CTC_ORGSPEC_VARINSTANT_LEN20)))
            {
                /* Ignore the following variables */
                *pReplyLen = (pReplyPtr - pReplyBuf);
                return EPON_OAM_ERR_OK;
            }
            varInstant.varDesc = varContainer.varDesc;
            varInstant.varWidth = CTC_ORGSPEC_VARINSTANT_WIDTH20;
            memcpy(varInstant.varData, pProcPtr, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH20);
            CTC_BUF_ADD(pProcPtr, remainLen, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH20); /* Data */
            /* Set instant pointer for all remain variable request */
            pVarInstant = &varInstant;
            /* Encode the variable instant into reply packet */
            ctc_oam_orgSpecVarInstant_gen(pVarInstant, pReplyPtr, &genLen);
            CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
            break;
        case CTC_VAR_REQBRANCH_STDATTR:
        case CTC_VAR_REQBRANCH_EXTATTR:
        case CTC_VAR_REQBRANCH_STDACT:
        case CTC_VAR_REQBRANCH_EXTACT:
            if(remainLen < varContainer.varWidth)
            {
                /* Insufficient length for parsing
                 * Skip all remain frame
                 */
                *pReplyLen = (pReplyPtr - pReplyBuf);
                return EPON_OAM_ERR_OK;
            }

            ret = ctc_oam_orgSepcVarCb_get(varContainer.varDesc, &varCb);
            if((EPON_OAM_ERR_OK != ret) ||
               (NULL == varCb.handler_set) ||
               (0 == (varCb.op & CTC_VAR_OP_SET)))
            {
                /* Ignore unsupport branch/leaf 
                 * Skip the data field of the frame
                 */
                CTC_BUF_ADD(pProcPtr, remainLen, varContainer.varWidth); /* Data */
                break;
            }

            /* Extract data field from frame */
            if(0 != varContainer.varWidth)
            {
                varContainer.pVarData = (unsigned char *) malloc(varContainer.varWidth);
                if(NULL == varContainer.pVarData)
                {
                    return EPON_OAM_ERR_MEM;
                }
                memcpy(varContainer.pVarData, pProcPtr, varContainer.varWidth);
                CTC_BUF_ADD(pProcPtr, remainLen, varContainer.varWidth); /* Data */
            }
            else
            {
                varContainer.pVarData = NULL;
            }

            epon_oam_dbgExt_get(&extFlag);
            if(extFlag & EPON_OAM_DBGEXT_CTC_VARSET)
            {
                /* Use dummy set to ignore all actual set functions */
                ret = ctc_oam_varCb_dummy_set(llidIdx, CTC_VAR_OP_SET, pVarInstant, varContainer.varDesc, &varContainer);
            }
            else
            {
                /* Use branch/leaf specific callback function to process */
                ret = varCb.handler_set(llidIdx, CTC_VAR_OP_SET, pVarInstant, varContainer.varDesc, &varContainer);
            }
            if(EPON_OAM_ERR_OK == ret)
            {
                if(bufLen >= CTC_ORGSPEC_VARCONTAINER_MIN)
                {
                    /* For set operation, only set result should be encoded */
                    varContainer.varWidth |= CTC_VAR_CONTAINER_INDICATOR;

                    genLen = 0;
                    ctc_oam_orgSpecVarContainer_gen(
                        &varContainer,
                        pReplyPtr,
                        &genLen);
                    CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
                }
            }

            if(NULL != varContainer.pVarData)
            {
                free(varContainer.pVarData);
            }
            break;
        case CTC_VAR_REQBRANCH_END:
            *pReplyLen = (pReplyPtr - pReplyBuf);
            return EPON_OAM_ERR_OK;
        default:
            break;
        }
    }
    *pReplyLen = (pReplyPtr - pReplyBuf);

    return EPON_OAM_ERR_OK;
}
#endif /* CTC_OAM_SUPPORT_BENEATH_20 */

#if CTC_OAM_SUPPORT_ABOVE_21
static int ctc_oam_orgSpecValidInstantLeaf_decode21(
    ctc_varInstant_t *pVarInstant)
{
    unsigned int value;

    switch(pVarInstant->varDesc.varLeaf)
    {
        case CTC_VAR_REQBRANCH_INSTANT_PORT:
            CTC_BUF_PARSE32(pVarInstant->varData, &value);
            pVarInstant->parse.uniPort.portType = (value >> 24) & 0xffUL;
            pVarInstant->parse.uniPort.chassisNo = (value >> 22) & 0x03UL;
            pVarInstant->parse.uniPort.slotNo = (value >> 16) & 0x3fUL;
            pVarInstant->parse.uniPort.portNo = value & 0xffUL;
            /* Valid instant leaf value for CTC 2.1 */
            return 1;
        case CTC_VAR_REQBRANCH_INSTANT_LLID:
            CTC_BUF_PARSE32(pVarInstant->varData, &pVarInstant->parse.llid);
            /* Valid instant leaf value for CTC 2.1 */
            return 1;
        case CTC_VAR_REQBRANCH_INSTANT_PONIF:
            CTC_BUF_PARSE32(pVarInstant->varData, &pVarInstant->parse.ponIf);
            /* Valid instant leaf value for CTC 2.1 */
            return 1;
        case CTC_VAR_REQBRANCH_INSTANT_ONU:
        default:
            return 0;
    }
}

/* For CTC version larger or equal to 2.1 */
static int ctc_oam_orgSpecVar_req21(
    unsigned char llidIdx,      /* LLID index of the incoming operation */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    int ret;
    unsigned char *pProcPtr, *pReplyPtr;
    unsigned short genLen;
    ctc_varCb_t varCb;
    ctc_varDescriptor_t varDesc;
    ctc_varInstant_t *pVarInstant = NULL, varInstant;
    ctc_varContainer_t varContainer;
    unsigned short remainLen;

    /* 1. Parse branch/leaf and search database for process function
     * 2. Call process function to preocess the branch/leaf
     */
    pProcPtr = pFrame;
    pReplyPtr = pReplyBuf;
    remainLen = length;
    *pReplyLen = 0;

    while(remainLen >= CTC_ORGSPEC_VARDESC_LEN)
    {
        varDesc.varBranch = *pProcPtr;
        memcpy(&varDesc.varLeaf, &pProcPtr[1], sizeof(unsigned short));
        CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_VARDESC_LEN); /* Variable Descriptor */

        switch(varDesc.varBranch)
        {
        case CTC_VAR_REQBRANCH_INSTANT21:
            /* Parse for variable instant */
            if((remainLen < CTC_ORGSPEC_VARINSTANT_LEN21) &&
               (bufLen > (CTC_ORGSPEC_VARDESC_LEN + CTC_ORGSPEC_VARINSTANT_LEN21)))
            {
                /* Ignore the following variables */
                *pReplyLen = (pReplyPtr - pReplyBuf);
                return EPON_OAM_ERR_OK;
            }
            varInstant.varDesc = varDesc;
            varInstant.varWidth = CTC_ORGSPEC_VARINSTANT_WIDTH21;
            CTC_BUF_ADD(pProcPtr, remainLen, 1); /* Width */
            memcpy(varInstant.varData, pProcPtr, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH21);
            CTC_BUF_ADD(pProcPtr, remainLen, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH21); /* Data */
            if(ctc_oam_orgSpecValidInstantLeaf_decode21(&varInstant))
            {
                /* Set instant pointer for all remain variable request */
                pVarInstant = &varInstant;
                if(!CTC_INSTANT_IS_ALLPORTS(pVarInstant))
                {
                    /* Encode the variable instant into reply packet */
                    ctc_oam_orgSpecVarInstant_gen(pVarInstant, pReplyPtr, &genLen);
                    CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
                }
                else
                {
                    /* For get request with ALL port instatnt, all port should response with its own data and instant 
                     * The response callback should encode the instant for each port
                     */
                }
            }
            else
            {
                /* Ignore the invalid variable instant */
                pVarInstant = NULL;
            }
            break;
        case CTC_VAR_REQBRANCH_STDATTR:
        case CTC_VAR_REQBRANCH_EXTATTR:
        case CTC_VAR_REQBRANCH_STDACT:
        case CTC_VAR_REQBRANCH_EXTACT:
            
            ret = ctc_oam_orgSepcVarCb_get(varDesc, &varCb);
            if((EPON_OAM_ERR_OK != ret) ||
               (NULL == varCb.handler_get) ||
               (0 == (varCb.op & CTC_VAR_OP_GET)))
            {
                /* Ignore unsupport branch/leaf */
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_VARGET,
                    "[OAM:%s:%d] var get 0x%02x/0x%04x\n", __FILE__, __LINE__, varDesc.varBranch, varDesc.varLeaf);
                break;
            }
            else
            {
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_VARGET,
                    "[OAM:%s:%d] var get 0x%02x/0x%04x %s\n", __FILE__, __LINE__, varDesc.varBranch, varDesc.varLeaf, varCb.varName);
            }

            /* Use branch/leaf specific callback function to process */
            varContainer.pVarData = NULL;
            ret = varCb.handler_get(llidIdx, CTC_VAR_OP_GET, pVarInstant, varDesc, &varContainer);
            if(EPON_OAM_ERR_OK == ret)
            {
                if(bufLen >= (CTC_ORGSPEC_VARDESC_LEN + varContainer.varWidth + 1 /* Width */))
                {
                    genLen = 0;
                    ctc_oam_orgSpecVarContainer_gen(
                        &varContainer,
                        pReplyPtr,
                        &genLen);
                    CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
                }
            }

            if(NULL != varContainer.pVarData)
            {
                free(varContainer.pVarData);
            }
            break;
        case CTC_VAR_REQBRANCH_END:
            *pReplyLen = (pReplyPtr - pReplyBuf);
            return EPON_OAM_ERR_OK;
        default:
            break;
        }
    }
    *pReplyLen = (pReplyPtr - pReplyBuf);

    return EPON_OAM_ERR_OK;
}

/* For CTC version larger or equal to 2.1 */
static int ctc_oam_orgSpecSet_req21(
    unsigned char llidIdx,      /* LLID index of the incoming operation */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    int ret;
    unsigned int extFlag;
    unsigned char *pProcPtr, *pReplyPtr;
    unsigned short genLen;
    unsigned char noWidth;
    ctc_varCb_t varCb;
    ctc_varInstant_t *pVarInstant = NULL, varInstant;
    ctc_varContainer_t varContainer;
    unsigned short remainLen, cascadeLen, concateLen;

    /* 1. Parse branch/leaf and search database for process function
     * 2. Call process function to preocess the branch/leaf
     */
    pProcPtr = pFrame;
    pReplyPtr = pReplyBuf;
    remainLen = length;
    *pReplyLen = 0;

    while(remainLen >= CTC_ORGSPEC_VARDESC_LEN)
    {
        varContainer.varDesc.varBranch = *pProcPtr;
        memcpy(&varContainer.varDesc.varLeaf, &pProcPtr[1], sizeof(unsigned short));
        ctc_oam_orgSepcVarNoWidth_get(varContainer.varDesc, &noWidth);
        if(noWidth)
        {
            CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_VARDESC_LEN); /* Variable Descriptor */
            varContainer.varWidth = 0;
        }
        else
        {
            if(remainLen < (CTC_ORGSPEC_VARCONTAINER_MIN - CTC_ORGSPEC_VARDESC_LEN))
            {
                /* Insufficient length for parsing
                 * Skip all remain frame
                 */
                *pReplyLen = (pReplyPtr - pReplyBuf);
                return EPON_OAM_ERR_OK;
            }
            varContainer.varWidth = pProcPtr[3];
            /* Mark the next line, when olt reboot onu, the varContainer have no the member or the varWidth = 0 */
            varContainer.varWidth = (0 == varContainer.varWidth) ? 0x80 : varContainer.varWidth;
            CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_VARCONTAINER_MIN); /* Variable Container */
        }

        switch(varContainer.varDesc.varBranch)
        {
        case CTC_VAR_REQBRANCH_INSTANT21:
            /* Parse for variable instant */
            if((remainLen < varContainer.varWidth) &&
               (bufLen > (CTC_ORGSPEC_VARDESC_LEN + CTC_ORGSPEC_VARINSTANT_LEN21)))
            {
                /* Ignore the following variables */
                *pReplyLen = (pReplyPtr - pReplyBuf);
                return EPON_OAM_ERR_OK;
            }
            varInstant.varDesc = varContainer.varDesc;
            varInstant.varWidth = CTC_ORGSPEC_VARINSTANT_WIDTH21;
            memcpy(varInstant.varData, pProcPtr, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH21);
            CTC_BUF_ADD(pProcPtr, remainLen, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH21); /* Data */
            if(ctc_oam_orgSpecValidInstantLeaf_decode21(&varInstant))
            {
                /* Set instant pointer for all remain variable request */
                pVarInstant = &varInstant;
                /* Encode the variable instant into reply packet */
                ctc_oam_orgSpecVarInstant_gen(pVarInstant, pReplyPtr, &genLen);
                CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
            }
            else
            {
                /* Ignore the invalid variable instant */
                pVarInstant = NULL;
            }
            break;
        case CTC_VAR_REQBRANCH_STDATTR:
        case CTC_VAR_REQBRANCH_EXTATTR:
        case CTC_VAR_REQBRANCH_STDACT:
        case CTC_VAR_REQBRANCH_EXTACT:
            if(remainLen < varContainer.varWidth)
            {
                /* Insufficient length for parsing
                 * Skip all remain frame
                 */
                *pReplyLen = (pReplyPtr - pReplyBuf);
                return EPON_OAM_ERR_OK;
            }

            ret = ctc_oam_orgSepcVarCb_get(varContainer.varDesc, &varCb);
            if((EPON_OAM_ERR_OK != ret) ||
               (NULL == varCb.handler_set) ||
               (0 == (varCb.op & CTC_VAR_OP_SET)))
            {
                /* Ignore unsupport branch/leaf 
                 * Skip the data field of the frame
                 */
                CTC_BUF_ADD(pProcPtr, remainLen, varContainer.varWidth); /* Data */
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_VARSET,
                    "[OAM:%s:%d] var set 0x%02x/0x%04x\n", __FILE__, __LINE__, varContainer.varDesc.varBranch, varContainer.varDesc.varLeaf);
                break;
            }
            else
            {
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_VARSET,
                    "[OAM:%s:%d] var set 0x%02x/0x%04x %s\n", __FILE__, __LINE__, varContainer.varDesc.varBranch, varContainer.varDesc.varLeaf, varCb.varName);
            }

            /* Extract data field from frame */
            if(0 != varContainer.varWidth)
            {
                if(varContainer.varWidth < 0x80)
                {
                    /* No cascade variable container is possible */
                    varContainer.pVarData = (unsigned char *) malloc(varContainer.varWidth);
                    if(NULL == varContainer.pVarData)
                    {
                        return EPON_OAM_ERR_MEM;
                    }
                    memcpy(varContainer.pVarData, pProcPtr, varContainer.varWidth);
                    CTC_BUF_ADD(pProcPtr, remainLen, varContainer.varWidth); /* Data */
                }
                else
                {
                    /* Cascade all data fields of the same branch/leaf */
                    /* Retrive overall length */
                    ret = ctc_oam_orgSepcVarCascadeLen_get(
                        varContainer.varDesc,
                        pProcPtr - CTC_ORGSPEC_VARCONTAINER_MIN, /* pProcPtr has been added CTC_ORGSPEC_VARCONTAINER_MIN */
                        remainLen + CTC_ORGSPEC_VARCONTAINER_MIN,/* remainLen has been decreased CTC_ORGSPEC_VARCONTAINER_MIN */
                        &cascadeLen);
                    if((ret != EPON_OAM_ERR_OK) || (0 == cascadeLen))
                    {
                        varContainer.pVarData = NULL;
                    }
                    else
                    {
                        varContainer.pVarData = (unsigned char *) malloc(cascadeLen);
                        varContainer.varWidth = cascadeLen;
                        if(NULL == varContainer.pVarData)
                        {
                            return EPON_OAM_ERR_MEM;
                        }
                        memcpy(varContainer.pVarData, pProcPtr, 0x80);
                        CTC_BUF_ADD(pProcPtr, remainLen, 0x80); /* Data */
                        concateLen = 0x80;
                        cascadeLen -= 0x80;

                        while(cascadeLen > 0)
                        {
                            CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_VARCONTAINER_MIN); /* Variable Container */
                            if(cascadeLen > 0x80)
                            {
                                memcpy(varContainer.pVarData + concateLen, pProcPtr, 0x80);
                                CTC_BUF_ADD(pProcPtr, remainLen, 0x80); /* Data */
                                concateLen += 0x80;
                                cascadeLen -= 0x80;
                            }
                            else
                            {
                                memcpy(varContainer.pVarData + concateLen, pProcPtr, cascadeLen);
                                CTC_BUF_ADD(pProcPtr, remainLen, cascadeLen); /* Data */
                                concateLen += cascadeLen;
                                cascadeLen = 0;
                            }
                        }
                    }
                }
            }
            else
            {
                varContainer.pVarData = NULL;
            }

            epon_oam_dbgExt_get(&extFlag);
            if(extFlag & EPON_OAM_DBGEXT_CTC_VARSET)
            {
                /* Use dummy set to ignore all actual set functions */
                ret = ctc_oam_varCb_dummy_set(llidIdx, CTC_VAR_OP_SET, pVarInstant, varContainer.varDesc, &varContainer);
            }
            else
            {
                /* Use branch/leaf specific callback function to process */
                ret = varCb.handler_set(llidIdx, CTC_VAR_OP_SET, pVarInstant, varContainer.varDesc, &varContainer);
            }
            if(EPON_OAM_ERR_OK == ret)
            {
                if(bufLen >= CTC_ORGSPEC_VARCONTAINER_MIN)
                {
                    /* For set operation, only set result should be encoded */
                    varContainer.varWidth |= CTC_VAR_CONTAINER_INDICATOR;

                    genLen = 0;
                    ctc_oam_orgSpecVarContainer_gen(
                        &varContainer,
                        pReplyPtr,
                        &genLen);
                    CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
                }
            }

            if(NULL != varContainer.pVarData)
            {
                free(varContainer.pVarData);
            }
            break;
        case CTC_VAR_REQBRANCH_END:
            *pReplyLen = (pReplyPtr - pReplyBuf);
            return EPON_OAM_ERR_OK;
        default:
            break;
        }
    }
    *pReplyLen = (pReplyPtr - pReplyBuf);

    return EPON_OAM_ERR_OK;
}
#endif /* CTC_OAM_SUPPORT_ABOVE_21 */
#else
static int ctc_oam_orgSpecValidInstantLeaf_decode20(
    ctc_varInstant_t *pVarInstant)
{
    unsigned int value;

    switch(pVarInstant->varDesc.varLeaf)
    {
        case CTC_VAR_REQBRANCH_INSTANT_PORT:
            value = *pVarInstant->varData;
            /* Chassis & slot number are not defined in CTC 2.0- instant, always zero */
            pVarInstant->parse.uniPort.chassisNo = 0;
            pVarInstant->parse.uniPort.slotNo = 0;
            if(0 == value)
            {
                pVarInstant->parse.uniPort.portType = CTC_VAR_INSTANT_PORTTYPE_PON;
                pVarInstant->parse.uniPort.portNo = 0;
            }
            else if(value <= 0x4F)
            {
                pVarInstant->parse.uniPort.portType = CTC_VAR_INSTANT_PORTTYPE_ETHERNET;
                pVarInstant->parse.uniPort.portNo = value;
            }
            else if(value <= 0x8F)
            {
                pVarInstant->parse.uniPort.portType = CTC_VAR_INSTANT_PORTTYPE_VOIP;
                pVarInstant->parse.uniPort.portNo = value - 0x4F;
            }
            else if(value <= 0x9F)
            {
                pVarInstant->parse.uniPort.portType = CTC_VAR_INSTANT_PORTTYPE_E1;
                pVarInstant->parse.uniPort.portNo = value - 0x90;
            }
            else if(value < 0xFF)
            {
                pVarInstant->parse.uniPort.portType = CTC_VAR_INSTANT_PORTTYPE_OTHERS;
                pVarInstant->parse.uniPort.portNo = value - 0x9F;
            }
            else if(value == 0xFF)
            {
                pVarInstant->parse.uniPort.portType = CTC_VAR_INSTANT_PORTTYPE_ETHERNET;
                pVarInstant->parse.uniPort.portNo = CTC_VAR_INSTANT_PORTALL;
            }
            else
            {
                return 0;
            }
            /* Valid instant leaf value for CTC 2.0 */
            return 1;
        default:
            return 0;
    }
}

static int ctc_oam_orgSpecValidInstantLeaf_decode21(
    ctc_varInstant_t *pVarInstant)
{
    unsigned int value;

    switch(pVarInstant->varDesc.varLeaf)
    {
        case CTC_VAR_REQBRANCH_INSTANT_PORT:
            CTC_BUF_PARSE32(pVarInstant->varData, &value);
            pVarInstant->parse.uniPort.portType = (value >> 24) & 0xffUL;
            pVarInstant->parse.uniPort.chassisNo = (value >> 22) & 0x03UL;
            pVarInstant->parse.uniPort.slotNo = (value >> 16) & 0x3fUL;
            pVarInstant->parse.uniPort.portNo = value & 0xffffUL;
            /* Valid instant leaf value for CTC 2.1 */
            return 1;
        case CTC_VAR_REQBRANCH_INSTANT_LLID:
            CTC_BUF_PARSE32(pVarInstant->varData, &value);
			pVarInstant->parse.llid = value & 0xffffUL;
            /* Valid instant leaf value for CTC 2.1 */
            return 1;
        case CTC_VAR_REQBRANCH_INSTANT_PONIF:
            CTC_BUF_PARSE32(pVarInstant->varData, &value);
			pVarInstant->parse.ponIf = value & 0x1UL;
            /* Valid instant leaf value for CTC 2.1 */
            return 1;
        case CTC_VAR_REQBRANCH_INSTANT_ONU:
        default:
            return 0;
    }
}

/* process oam get request command with extra parameter 
   now support command (0xC7/0x0082 - AlarmThreshold)
   return value: 1: no extra parameter ignore, 0: has extra parameter, <0: process error,ignore
 */
static int ctc_oam_process_request_with_parameter(
	unsigned char *pProcPtr,
	ctc_varDescriptor_t varDesc,
	ctc_varContainer_t *pVarContainer)
{
	unsigned short varWidth;
	if((varDesc.varBranch == CTC_VAR_REQBRANCH_EXTATTR) && (varDesc.varLeaf == 0x0082))
	{
		varWidth = pProcPtr[0];
		EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_VARGET, "%s var get 0x%02x/0x%04x varWidth[%d]\n", 
					__func__, varDesc.varBranch, varDesc.varLeaf, varWidth);
		if(varWidth != CTC_OAM_VAR_ALARMTHRESHOLD_GET_LEN)
			return EPON_OAM_ERR_UNKNOWN;
		
        pVarContainer->pVarData = (unsigned char *) malloc(varWidth);
        if(NULL == pVarContainer->pVarData)
        {
           	return EPON_OAM_ERR_MEM;
        }
        memcpy(pVarContainer->pVarData, pProcPtr+1, varWidth);
        pVarContainer->varWidth = CTC_OAM_VAR_ALARMTHRESHOLD_GET_LEN; 
		return EPON_OAM_ERR_OK;
	}
	return 1;
}

/* For all CTC versions */
static int ctc_oam_orgSpecVar_req(
    unsigned char llidIdx,      /* LLID index of the incoming operation */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    int ret;
    unsigned char *pProcPtr, *pReplyPtr;
    unsigned short genLen;
    ctc_varCb_t varCb;
    ctc_varDescriptor_t varDesc;
    ctc_varInstant_t *pVarInstant = NULL, varInstant, varInstCnt;
    ctc_varContainer_t varContainer;
    unsigned short remainLen;

    /* 1. Parse branch/leaf and search database for process function
     * 2. Call process function to preocess the branch/leaf
     */
    pProcPtr = pFrame;
    pReplyPtr = pReplyBuf;
    remainLen = length;
    *pReplyLen = 0;

    while(remainLen >= CTC_ORGSPEC_VARDESC_LEN)
    {
        varDesc.varBranch = *pProcPtr;
#ifdef SUPPORT_OAM_BYTEORDER
		CTC_BUF_PARSE16(&pProcPtr[1], &varDesc.varLeaf);
#else
        memcpy(&varDesc.varLeaf, &pProcPtr[1], sizeof(unsigned short));
#endif
        CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_VARDESC_LEN); /* Variable Descriptor */

        switch(varDesc.varBranch)
        {
        case CTC_VAR_REQBRANCH_INSTANT20:
            /* Parse for variable instant */
            if((remainLen < CTC_ORGSPEC_VARINSTANT_LEN20) &&
               (bufLen > (CTC_ORGSPEC_VARDESC_LEN + CTC_ORGSPEC_VARINSTANT_LEN20)))
            {
                /* Ignore the following variables */
                *pReplyLen = (pReplyPtr - pReplyBuf);
                return EPON_OAM_ERR_OK;
            }
            varInstant.varDesc = varDesc;
            varInstant.varWidth = CTC_ORGSPEC_VARINSTANT_WIDTH20;
            CTC_BUF_ADD(pProcPtr, remainLen, 1); /* Width */
            memcpy(varInstant.varData, pProcPtr, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH20);
            CTC_BUF_ADD(pProcPtr, remainLen, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH20); /* Data */
            if(ctc_oam_orgSpecValidInstantLeaf_decode20(&varInstant))
            {
                /* Set instant pointer for all remain variable request */
                pVarInstant = &varInstant;
                if(!CTC_INSTANT_IS_ALLPORTS(pVarInstant))
                {
                    /* Encode the variable instant into reply packet */
                    ctc_oam_orgSpecVarInstant_gen(pVarInstant, pReplyPtr, &genLen);
                    CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
                }
                else
                {
                    /* For get request with ALL port instatnt, all port should response with its own data and instant 
                     * The response callback should encode the instant for each port
                     */
                }
            }
            else
            {
                /* Ignore the invalid variable instant */
                pVarInstant = NULL;
            }
            break;
        case CTC_VAR_REQBRANCH_INSTANT21:
            /* Parse for variable instant */
            if((remainLen < CTC_ORGSPEC_VARINSTANT_LEN21) &&
               (bufLen > (CTC_ORGSPEC_VARDESC_LEN + CTC_ORGSPEC_VARINSTANT_LEN21)))
            {
                /* Ignore the following variables */
                *pReplyLen = (pReplyPtr - pReplyBuf);
                return EPON_OAM_ERR_OK;
            }
            varInstant.varDesc = varDesc;
            varInstant.varWidth = CTC_ORGSPEC_VARINSTANT_WIDTH21;
            CTC_BUF_ADD(pProcPtr, remainLen, 1); /* Width */
            memcpy(varInstant.varData, pProcPtr, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH21);
            CTC_BUF_ADD(pProcPtr, remainLen, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH21); /* Data */
            if(ctc_oam_orgSpecValidInstantLeaf_decode21(&varInstant))
            {
                /* Set instant pointer for all remain variable request */
                pVarInstant = &varInstant;
                if(!CTC_INSTANT_IS_ALLPORTS(pVarInstant))
                {
                    /* Encode the variable instant into reply packet */
                    ctc_oam_orgSpecVarInstant_gen(pVarInstant, pReplyPtr, &genLen);
                    CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
                }
                else
                {
                    /* For get request with ALL port instatnt, all port should response with its own data and instant 
                     * The response should encode the instant for each port
                     */
                }
            }
            else
            {
                /* Ignore the invalid variable instant */
                pVarInstant = NULL;
            }
            break;
        case CTC_VAR_REQBRANCH_STDATTR:
        case CTC_VAR_REQBRANCH_EXTATTR:
        case CTC_VAR_REQBRANCH_STDACT:
        case CTC_VAR_REQBRANCH_EXTACT:
            
            ret = ctc_oam_orgSepcVarCb_get(varDesc, &varCb);
            if((EPON_OAM_ERR_OK != ret) ||
               (NULL == varCb.handler_get) ||
               (0 == (varCb.op & CTC_VAR_OP_GET)))
            {
                /* Ignore unsupport branch/leaf */
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_VARGET,
                    "[OAM:%s:%d] var get 0x%02x/0x%04x\n", __FILE__, __LINE__, varDesc.varBranch, varDesc.varLeaf);
                break;
            }
            else
            {
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_VARGET,
                    "[OAM:%s:%d] var get 0x%02x/0x%04x %s\n", __FILE__, __LINE__, varDesc.varBranch, varDesc.varLeaf, varCb.varName);
            }

            if((NULL != pVarInstant) && (CTC_INSTANT_IS_ALLPORTS(pVarInstant)))
            {
                varInstCnt = *pVarInstant;
                /* If the target instant is all ports, loop each port one by one */
                for(varInstCnt.parse.uniPort.portNo = 1 ; varInstCnt.parse.uniPort.portNo < CTC_VAR_INSTANT_PORTALL ; varInstCnt.parse.uniPort.portNo++)
                {
                    /* Use branch/leaf specific callback function to process */
                    varContainer.pVarData = NULL;

					if(ctc_oam_process_request_with_parameter(pProcPtr, varDesc, &varContainer) == EPON_OAM_ERR_OK)
					{
						CTC_BUF_ADD(pProcPtr, remainLen, 1+varContainer.varWidth); /*Width(1) + parameter length*/
					}
					
                    ret = varCb.handler_get(llidIdx, CTC_VAR_OP_GET, &varInstCnt, varDesc, &varContainer);
                    if(EPON_OAM_ERR_OK == ret)
                    {
                        if(bufLen >= (CTC_ORGSPEC_VARDESC_LEN + CTC_ORGSPEC_VARINSTANT_LEN21))
                        {
                            /* Update port to the varData */
                            *((unsigned short *)&varInstCnt.varData[2]) = varInstCnt.parse.uniPort.portNo;
                            /* Encode the variable instant into reply packet */
                            ctc_oam_orgSpecVarInstant_gen(&varInstCnt, pReplyPtr, &genLen);
                            CTC_BUF_ADD(pReplyPtr, bufLen, genLen);

                            if(bufLen >= (CTC_ORGSPEC_VARDESC_LEN + varContainer.varWidth + 1 /* Width */))
                            {
                                genLen = 0;
                                ctc_oam_orgSpecVarContainer_gen(
                                    &varContainer,
                                    pReplyPtr,
                                    &genLen);
                                CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
                            }
                        }
                    }
                    if(NULL != varContainer.pVarData)
                    {
                        free(varContainer.pVarData);
                    }
                    if(EPON_OAM_ERR_OK != ret)
                    {
                        /* Stop whenever the handler return error */
                        break;
                    }
                }
            }
            else
            {
                /* Use branch/leaf specific callback function to process */
                varContainer.pVarData = NULL;

				if(ctc_oam_process_request_with_parameter(pProcPtr, varDesc, &varContainer) == EPON_OAM_ERR_OK)
				{
					CTC_BUF_ADD(pProcPtr, remainLen, 1+varContainer.varWidth); /*Width(1) + parameter length*/
				}
				
                ret = varCb.handler_get(llidIdx, CTC_VAR_OP_GET, pVarInstant, varDesc, &varContainer);
                if(EPON_OAM_ERR_OK == ret)
                {
                    if(bufLen >= (CTC_ORGSPEC_VARDESC_LEN + varContainer.varWidth + 1 /* Width */))
                    {
                        genLen = 0;
                        ctc_oam_orgSpecVarContainer_gen(
                            &varContainer,
                            pReplyPtr,
                            &genLen);
                        CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
                    }
                }
                if(NULL != varContainer.pVarData)
                {
                    free(varContainer.pVarData);
                }
            }

            break;
        case CTC_VAR_REQBRANCH_END:
            *pReplyLen = (pReplyPtr - pReplyBuf);
            return EPON_OAM_ERR_OK;
        default:
            break;
        }
    }
    *pReplyLen = (pReplyPtr - pReplyBuf);

    return EPON_OAM_ERR_OK;
}

/* For all CTC versions */
static int ctc_oam_orgSpecSet_req(
    unsigned char llidIdx,      /* LLID index of the incoming operation */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    int ret;
    unsigned int extFlag;
    unsigned char *pProcPtr, *pReplyPtr;
    unsigned short genLen;
    unsigned char noWidth;
    ctc_varCb_t varCb;
    ctc_varInstant_t *pVarInstant = NULL, varInstant;
    ctc_varContainer_t varContainer;
    unsigned short remainLen, cascadeLen, concateLen;

    /* 1. Parse branch/leaf and search database for process function
     * 2. Call process function to preocess the branch/leaf
     */
    pProcPtr = pFrame;
    pReplyPtr = pReplyBuf;
    remainLen = length;
    *pReplyLen = 0;

    while(remainLen >= CTC_ORGSPEC_VARDESC_LEN)
    {
        varContainer.varDesc.varBranch = *pProcPtr;
#ifdef SUPPORT_OAM_BYTEORDER
		CTC_BUF_PARSE16(&pProcPtr[1], &varContainer.varDesc.varLeaf);
#else
        memcpy(&varContainer.varDesc.varLeaf, &pProcPtr[1], sizeof(unsigned short));
#endif
        ctc_oam_orgSepcVarNoWidth_get(varContainer.varDesc, &noWidth);
        if(noWidth)
        {
            CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_VARDESC_LEN); /* Variable Descriptor */
            varContainer.varWidth = 0;
        }
        else
        {
            if(remainLen < (CTC_ORGSPEC_VARCONTAINER_MIN - CTC_ORGSPEC_VARDESC_LEN))
            {
                /* Insufficient length for parsing
                 * Skip all remain frame
                 */
                *pReplyLen = (pReplyPtr - pReplyBuf);
                return EPON_OAM_ERR_OK;
            }
            varContainer.varWidth = pProcPtr[3];
			CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_VARCONTAINER_MIN); /* Variable Container */
			
            /* Mark the next line, when olt reboot onu, the varContainer have no the member or the varWidth = 0 */
			if(0 == varContainer.varWidth)
			{	/* fix error: varContainer.varWidth is larger than remainLen when remainLen is less than 0x80 */
				varContainer.varWidth = (remainLen < 0x80) ? remainLen : 0x80;
				EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_VARSET, "[OAM:%s:%d] remainLen(%d) varWidth(%d)\n", 
                                __FILE__, __LINE__, remainLen, varContainer.varWidth);
			}
        }

        switch(varContainer.varDesc.varBranch)
        {
        case CTC_VAR_REQBRANCH_INSTANT20:
            /* Parse for variable instant */
            if((remainLen < varContainer.varWidth) &&
               (bufLen > (CTC_ORGSPEC_VARDESC_LEN + CTC_ORGSPEC_VARINSTANT_LEN20)))
            {
                /* Ignore the following variables */
                *pReplyLen = (pReplyPtr - pReplyBuf);
                return EPON_OAM_ERR_OK;
            }
            varInstant.varDesc = varContainer.varDesc;
            varInstant.varWidth = CTC_ORGSPEC_VARINSTANT_WIDTH20;
            memcpy(varInstant.varData, pProcPtr, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH20);
            CTC_BUF_ADD(pProcPtr, remainLen, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH20); /* Data */
            if(ctc_oam_orgSpecValidInstantLeaf_decode20(&varInstant))
            {
                /* Set instant pointer for all remain variable request */
                pVarInstant = &varInstant;
                /* Encode the variable instant into reply packet */
                ctc_oam_orgSpecVarInstant_gen(pVarInstant, pReplyPtr, &genLen);
                CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
            }
            else
            {
                /* Ignore the invalid variable instant */
                pVarInstant = NULL;
            }
            break;
        case CTC_VAR_REQBRANCH_INSTANT21:
            /* Parse for variable instant */
            if((remainLen < varContainer.varWidth) &&
               (bufLen > (CTC_ORGSPEC_VARDESC_LEN + CTC_ORGSPEC_VARINSTANT_LEN21)))
            {
                /* Ignore the following variables */
                *pReplyLen = (pReplyPtr - pReplyBuf);
                return EPON_OAM_ERR_OK;
            }
            varInstant.varDesc = varContainer.varDesc;
            varInstant.varWidth = CTC_ORGSPEC_VARINSTANT_WIDTH21;
            memcpy(varInstant.varData, pProcPtr, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH21);
            CTC_BUF_ADD(pProcPtr, remainLen, sizeof(unsigned char) * CTC_ORGSPEC_VARINSTANT_WIDTH21); /* Data */
            if(ctc_oam_orgSpecValidInstantLeaf_decode21(&varInstant))
            {
                /* Set instant pointer for all remain variable request */
                pVarInstant = &varInstant;
                /* Encode the variable instant into reply packet */
                ctc_oam_orgSpecVarInstant_gen(pVarInstant, pReplyPtr, &genLen);
                CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
            }
            else
            {
                /* Ignore the invalid variable instant */
                pVarInstant = NULL;
            }
            break;
        case CTC_VAR_REQBRANCH_STDATTR:
        case CTC_VAR_REQBRANCH_EXTATTR:
        case CTC_VAR_REQBRANCH_STDACT:
        case CTC_VAR_REQBRANCH_EXTACT:
            if(remainLen < varContainer.varWidth)
            {
                /* Insufficient length for parsing
                 * Skip all remain frame
                 */
                *pReplyLen = (pReplyPtr - pReplyBuf);
				EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_VARSET, "[OAM:%s:%d] remainLen(%d) varWidth(%d)\n", 
                                __FILE__, __LINE__, remainLen, varContainer.varWidth);
                return EPON_OAM_ERR_OK;
            }

			/* decide if auth is successful when it is mac auth */
			if(authType[llidIdx] == CTC_OAM_ONUAUTH_AUTH_MAC)				
			{
				setCommandNum[llidIdx]++;
				if(setCommandNum[llidIdx] >= CTC_OAM_MAC_AUTH_SET_COMMAND_NUM)
				{
					authType[llidIdx] = CTC_OAM_ONUAUTH_AUTH_IGNORE;

					epon_oam_register_success_number_increase(llidIdx);
				#ifdef SUPPORT_OAM_EVENT_TO_KERNEL
					epon_oam_state_event_send(llidIdx, RTK_EPON_STATE_OAM_AUTH_SUCC);
				#endif
					ctc_oam_onuAuthSuccTime_set(llidIdx);
					ctc_onuAuth_state[llidIdx] = CTC_OAM_ONUAUTH_STATE_SUCC;
					epon_oam_event_send(llidIdx, EPON_OAM_EVENT_AUTH_SUCCESS);
				}
			}
			
            ret = ctc_oam_orgSepcVarCb_get(varContainer.varDesc, &varCb);
            if((EPON_OAM_ERR_OK != ret) ||
               (NULL == varCb.handler_set) ||
               (0 == (varCb.op & CTC_VAR_OP_SET)))
            {
                /* Ignore unsupport branch/leaf 
                 * Skip the data field of the frame
                 */
                CTC_BUF_ADD(pProcPtr, remainLen, varContainer.varWidth); /* Data */
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_VARSET,
                    "[OAM:%s:%d] var set 0x%02x/0x%04x\n", __FILE__, __LINE__, varContainer.varDesc.varBranch, varContainer.varDesc.varLeaf);

                /* siyuan 20170817: reply a fake SETOK response for any unsupported ctc set commands */
                if(bufLen >= CTC_ORGSPEC_VARCONTAINER_MIN)
                {
                    EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_VARSET, "[OAM:%s:%d] 0x%02x/0x%04x reply a fake SETOK response\n", 
                                __FILE__, __LINE__, varContainer.varDesc.varBranch, varContainer.varDesc.varLeaf);
                    /* For set operation, only set result should be encoded */
                    varContainer.varWidth = CTC_OAM_VAR_RESP_SETOK | CTC_VAR_CONTAINER_INDICATOR;

                    genLen = 0;
                    ctc_oam_orgSpecVarContainer_gen(
                        &varContainer,
                        pReplyPtr,
                        &genLen);
                    CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
                }
				break;
            }
            else
            {
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_VARSET,
                    "[OAM:%s:%d] var set 0x%02x/0x%04x %s\n", __FILE__, __LINE__, varContainer.varDesc.varBranch, varContainer.varDesc.varLeaf, varCb.varName);
            }

            /* Extract data field from frame */
            if(0 != varContainer.varWidth)
            {
                if(varContainer.varWidth < 0x80)
                {
                    /* No cascade variable container is possible */
                    varContainer.pVarData = (unsigned char *) malloc(varContainer.varWidth);
                    if(NULL == varContainer.pVarData)
                    {
                        return EPON_OAM_ERR_MEM;
                    }
                    memcpy(varContainer.pVarData, pProcPtr, varContainer.varWidth);
                    CTC_BUF_ADD(pProcPtr, remainLen, varContainer.varWidth); /* Data */
                }
                else
                {
                    /* Cascade all data fields of the same branch/leaf */
                    /* Retrive overall length */
                    ret = ctc_oam_orgSepcVarCascadeLen_get(
                        varContainer.varDesc,
                        pProcPtr - CTC_ORGSPEC_VARCONTAINER_MIN, /* pProcPtr has been added CTC_ORGSPEC_VARCONTAINER_MIN */
                        remainLen + CTC_ORGSPEC_VARCONTAINER_MIN,/* remainLen has been decreased CTC_ORGSPEC_VARCONTAINER_MIN */
                        &cascadeLen);
                    if((ret != EPON_OAM_ERR_OK) || (0 == cascadeLen))
                    {
                        varContainer.pVarData = NULL;
                    }
                    else
                    {
                        varContainer.pVarData = (unsigned char *) malloc(cascadeLen);
                        varContainer.varWidth = cascadeLen;
                        if(NULL == varContainer.pVarData)
                        {
                            return EPON_OAM_ERR_MEM;
                        }
                        memcpy(varContainer.pVarData, pProcPtr, 0x80);
                        CTC_BUF_ADD(pProcPtr, remainLen, 0x80); /* Data */
                        concateLen = 0x80;
                        cascadeLen -= 0x80;

                        while(cascadeLen > 0)
                        {
                            CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_VARCONTAINER_MIN); /* Variable Container */
                            if(cascadeLen > 0x80)
                            {
                                memcpy(varContainer.pVarData + concateLen, pProcPtr, 0x80);
                                CTC_BUF_ADD(pProcPtr, remainLen, 0x80); /* Data */
                                concateLen += 0x80;
                                cascadeLen -= 0x80;
                            }
                            else
                            {
                                memcpy(varContainer.pVarData + concateLen, pProcPtr, cascadeLen);
                                CTC_BUF_ADD(pProcPtr, remainLen, cascadeLen); /* Data */
                                concateLen += cascadeLen;
                                cascadeLen = 0;
                            }
                        }
                    }
                }
            }
            else
            {
                varContainer.pVarData = NULL;
            }

            epon_oam_dbgExt_get(&extFlag);
            if(extFlag & EPON_OAM_DBGEXT_CTC_VARSET)
            {
                /* Use dummy set to ignore all actual set functions */
                ret = ctc_oam_varCb_dummy_set(llidIdx, CTC_VAR_OP_SET, pVarInstant, varContainer.varDesc, &varContainer);
            }
            else
            {
                /* Use branch/leaf specific callback function to process */
                ret = varCb.handler_set(llidIdx, CTC_VAR_OP_SET, pVarInstant, varContainer.varDesc, &varContainer);
            }
            if(EPON_OAM_ERR_OK == ret)
            {
                if(bufLen >= CTC_ORGSPEC_VARCONTAINER_MIN)
                {
                    /* For set operation, only set result should be encoded */
                    varContainer.varWidth |= CTC_VAR_CONTAINER_INDICATOR;

                    genLen = 0;
                    ctc_oam_orgSpecVarContainer_gen(
                        &varContainer,
                        pReplyPtr,
                        &genLen);
                    CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
                }
            }
			else if(EPON_OAM_ERR_NO_RESPONSE == ret)
			{	
				/* Not reply to broadcast oam set request if this error occurred */
				return ret;
			}
			
            if(NULL != varContainer.pVarData)
            {
                free(varContainer.pVarData);
            }
            break;
        case CTC_VAR_REQBRANCH_END:
            *pReplyLen = (pReplyPtr - pReplyBuf);
            return EPON_OAM_ERR_OK;
        default:
            break;
        }
    }
    *pReplyLen = (pReplyPtr - pReplyBuf);

    return EPON_OAM_ERR_OK;
}
#endif

static int ctc_oam_orgSpecOnuAuth(
    unsigned char llidIdx,      /* LLID index of the incoming operation */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    int ret;
    unsigned char *pProcPtr, *pReplyPtr;
    unsigned short genLen;
    unsigned short remainLen;
    unsigned char authCode;
    unsigned char authType;
    unsigned char authFailType;
    unsigned short authDataLen;
    ctc_onuAuthLoid_t loidAuth;

    /* 1. Parse churning header
     * 2. Process CTC extended OAMPDU according to churning op code
     */
    pProcPtr = pFrame;
    pReplyPtr = pReplyBuf;
    remainLen = length;
    *pReplyLen = 0;

    if(remainLen < CTC_ORGSPEC_ONUAUTH_HDR_LEN)
    {
        /* Insufficient length for parsing */
        return EPON_OAM_ERR_OK;
    }

    /* Parse common header for ONU auth extended opcode */
    authCode = pProcPtr[0];
#ifdef SUPPORT_OAM_BYTEORDER
	CTC_BUF_PARSE16(&pProcPtr[1], &authDataLen);
#else
    ((unsigned char *)&authDataLen)[0] = pProcPtr[1];
    ((unsigned char *)&authDataLen)[1] = pProcPtr[2];
#endif
    CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_ONUAUTH_HDR_LEN);

    if(remainLen < authDataLen)
    {
        /* Insufficient length for parsing */
        return EPON_OAM_ERR_OK;
    }

    switch(authCode)
    {
    case CTC_ORGSPEC_ONUAUTH_CODE_REQ:
        rtk_pon_led_status_set(PON_LED_PON_MODE_EPON, PON_LED_STATE_AUTH_NG);
        
        authType = *pProcPtr;
        switch(authType)
        {
        case CTC_ORGSPEC_ONUAUTH_LOID:
            /* Reply the LOID + password */
            ret = ctc_oam_onuAuthLoid_get(llidIdx, &loidAuth);
            if(EPON_OAM_ERR_OK != ret ||
              (bufLen < CTC_ORGSPEC_ONUAUTH_HDR_LEN + CTC_ORGSPEC_ONUAUTH_RESP_LOID_LEN))
            {
                /* Get failed/insufficient length, don't reply to OLT */
                return EPON_OAM_ERR_ORG;
            }
            ctc_oam_orgSpecOnuAuthLoid_gen(&loidAuth, pReplyPtr, &genLen);
            CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
            break;
        default:
            /* Unsupported AUTH type */
            /* Reply the NAK */
            if(bufLen < CTC_ORGSPEC_ONUAUTH_HDR_LEN + CTC_ORGSPEC_ONUAUTH_RESP_NAK_LEN)
            {
                /* Insufficient length, don't reply to OLT */
                return EPON_OAM_ERR_ORG;
            }
            /* TODO - Only support LOID + password currently, add new auth type */
            ctc_oam_orgSpecOnuAuthNak_gen(CTC_ORGSPEC_ONUAUTH_LOID, pReplyPtr, &genLen);
            CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
            break;
        }
        break;
    case CTC_ORGSPEC_ONUAUTH_CODE_SUCC:
        ret = ctc_oam_onuAuthState_set(llidIdx, CTC_OAM_ONUAUTH_STATE_SUCC, 0);
        if(EPON_OAM_ERR_OK != ret)
        {
            /* Set failed */
            EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
                "[OAM:%s:%d] Fail to set auth state %u\n", __FILE__, __LINE__, ret);
            return EPON_OAM_ERR_OK;
        }
        break;
    case CTC_ORGSPEC_ONUAUTH_CODE_FAIL:
        authFailType = *pProcPtr;
        ret = ctc_oam_onuAuthState_set(llidIdx, CTC_OAM_ONUAUTH_STATE_FAIL, authFailType);
        if(EPON_OAM_ERR_OK != ret)
        {
            /* Set failed */
            EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
                "[OAM:%s:%d] Fail to set auth state %u\n", __FILE__, __LINE__, ret);
            return EPON_OAM_ERR_OK;
        }
        break;
    case CTC_ORGSPEC_ONUAUTH_CODE_RESP:
        /* Only ONU can send this, so there might be some error to receive this */
    default:
        return EPON_OAM_ERR_ORG;
    }
    *pReplyLen = (pReplyPtr - pReplyBuf);

    return EPON_OAM_ERR_OK;
}

static int ctc_oam_orgSpecSwDownload(
    unsigned char llidIdx,      /* LLID index of the incoming operation */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    int ret;
    unsigned int checkSize;
    unsigned char *pProcPtr, *pReplyPtr;
    unsigned char activeFlag, commitFlag;
    unsigned short genLen;
    unsigned short remainLen;
    ctc_swDownload_t inSwDownload, outSwDownload;
    unsigned int strLen, dataLen;
    FILE *pFp;
	int lastFileData = 0;
	struct timeval now;
	
    /* 1. Parse software download header
     * 2. Process CTC extended OAMPDU according to software download data type
     */
    pProcPtr = pFrame;
    pReplyPtr = pReplyBuf;
    remainLen = length;
    *pReplyLen = 0;

    if(remainLen < CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN)
    {
        /* Insufficient length for parsing */
        return EPON_OAM_ERR_OK;
    }

    /* Parse common header for churning extended opcode */
    inSwDownload.dataType = pProcPtr[0];
#ifdef SUPPORT_OAM_BYTEORDER
	CTC_BUF_PARSE16(&pProcPtr[1], &inSwDownload.length);
	CTC_BUF_PARSE16(&pProcPtr[3], &inSwDownload.tid);
	CTC_BUF_PARSE16(&pProcPtr[5], &inSwDownload.opCode);
#else
    ((unsigned char *)&inSwDownload.length)[0] = pProcPtr[1];
    ((unsigned char *)&inSwDownload.length)[1] = pProcPtr[2];
    ((unsigned char *)&inSwDownload.tid)[0] = pProcPtr[3];
    ((unsigned char *)&inSwDownload.tid)[1] = pProcPtr[4];
    ((unsigned char *)&inSwDownload.opCode)[0] = pProcPtr[5];
    ((unsigned char *)&inSwDownload.opCode)[1] = pProcPtr[6];
#endif
    CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN);

    EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
        "[OAM:%s:%d] software download, opcode %d\n", __FILE__, __LINE__, inSwDownload.opCode);

    switch(inSwDownload.dataType)
    {
    case CTC_ORGSPEC_SWDL_TYPE_FILE:
        switch(inSwDownload.opCode)
        {
        case CTC_ORGSPEC_SWDL_OPCODE_FILEREQ:
            /* Retrive file name */
            strLen = strlen(pProcPtr);
            if(strLen >= remainLen)
            {
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO, "[OAM:%s:%d]\n", __FILE__, __LINE__);
                return EPON_OAM_ERR_PARSE;
            }

            if(CTC_SWDL_STATE_GET() != CTC_OAM_SWDOWNLOAD_BUF_CLEAR)
            {
                /* Woops? Buffer is not empty? */
                ctc_oam_swDownloadBuf_clear();
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_WARN,
                    "[OAM:%s:%d] Clear software download buffer\n", __FILE__, __LINE__);
            }

            sem_wait(&ctcDataSem);
            ctc_swdl_file.fileName = (char *)malloc((strLen + 1) * sizeof(char));
            sem_post(&ctcDataSem);
            if(ctc_swdl_file.fileName == NULL)
            {
                ctc_oam_swDownloadError_alloc(inSwDownload, &outSwDownload, CTC_ORGSPEC_SWDL_ERROR_MEMORY, NULL);
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR, "[OAM:%s:%d]\n", __FILE__, __LINE__);
                break;
            }

            memcpy(ctc_swdl_file.fileName, pProcPtr, (strLen + 1) * sizeof(char));
            CTC_BUF_ADD(pProcPtr, remainLen, (strLen + 1));

            /* Retrive mode */
            strLen = strlen(pProcPtr);
            if(strLen > remainLen)
            {
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO, "[OAM:%s:%d]\n", __FILE__, __LINE__);
                return EPON_OAM_ERR_PARSE;
            }
            
            sem_wait(&ctcDataSem);
            ctc_swdl_file.mode = (char *)malloc((strLen + 1) * sizeof(char));
            sem_post(&ctcDataSem);
            if(ctc_swdl_file.mode == NULL)
            {
                free(ctc_swdl_file.fileName);
                sem_wait(&ctcDataSem);
                ctc_swdl_file.fileName = NULL;
                sem_post(&ctcDataSem);
                ctc_oam_swDownloadError_alloc(inSwDownload, &outSwDownload, CTC_ORGSPEC_SWDL_ERROR_MEMORY, NULL);
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR, "[OAM:%s:%d]\n", __FILE__, __LINE__);
                break;
            }

            memcpy(ctc_swdl_file.mode, pProcPtr, (strLen + 1) * sizeof(char));
            CTC_BUF_ADD(pProcPtr, remainLen, (strLen + 1));

            if(CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_FILEACK_LEN > bufLen)
            {
                /* Insufficient length, don't reply to OLT */
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR, "[OAM:%s:%d]\n", __FILE__, __LINE__);
                return EPON_OAM_ERR_ORG;
            }

            EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
                "[OAM:%s:%d] File write request received, file name %s, file mode %s\n",
                __FILE__, __LINE__, ctc_swdl_file.fileName, ctc_swdl_file.mode);

            CTC_SWDL_STATE_SET(CTC_OAM_SWDOWNLOAD_BUF_FILEREQ);

            /* File write request complete, generate the file transfer ACK */
            outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_FILE;
            outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_FILEACK;
            outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_FILEACK_LEN;
            outSwDownload.tid = inSwDownload.tid;
            outSwDownload.parse.fileAck.block = 0;
            break;
        case CTC_ORGSPEC_SWDL_OPCODE_FILEDATA:
            /* Retrive block # */
            if(remainLen < CTC_ORGSPEC_SWDOWNLOAD_FILEDATA_LEN)
            {
                return EPON_OAM_ERR_PARSE;
            }
            ((unsigned char *)&inSwDownload.parse.fileData.block)[0] = pProcPtr[0];
            ((unsigned char *)&inSwDownload.parse.fileData.block)[1] = pProcPtr[1];
            CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_SWDOWNLOAD_FILEDATA_LEN);
            
            EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
                "[OAM:%s:%d] block # %d, cur # %d\n",
                __FILE__, __LINE__, inSwDownload.parse.fileData.block, ctc_swdl_file.block);
            if(inSwDownload.parse.fileData.block <= ctc_swdl_file.block)
            {
                /* OLT resend? Ack the block again */
                outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_FILE;
                outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_FILEACK;
                outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_FILEACK_LEN;
                outSwDownload.tid = inSwDownload.tid;
                outSwDownload.parse.fileAck.block = inSwDownload.parse.fileData.block;
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO, "[OAM:%s:%d] resend ack for block # %d\n", __FILE__, __LINE__, inSwDownload.parse.fileData.block);
                break;
            }

            if(inSwDownload.parse.fileData.block != (ctc_swdl_file.block + 1))
            {
                /* block number not continue */
                ctc_oam_swDownloadError_alloc(inSwDownload, &outSwDownload, CTC_ORGSPEC_SWDL_ERROR_ILLEGALOAM, NULL);
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR, "[OAM:%s:%d] block # %d\n", __FILE__, __LINE__, inSwDownload.parse.fileData.block);
                break;
            }
            if((CTC_SWDL_STATE_GET() != CTC_OAM_SWDOWNLOAD_BUF_FILEREQ) &&
               (CTC_SWDL_STATE_GET() != CTC_OAM_SWDOWNLOAD_BUF_FILEDATA))
            {
                /* Incorrect buffer state */
                ctc_oam_swDownloadError_alloc(inSwDownload, &outSwDownload, CTC_ORGSPEC_SWDL_ERROR_ILLEGALOAM, NULL);
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR, "[OAM:%s:%d]\n", __FILE__, __LINE__);
                break;
            }
            CTC_SWDL_STATE_SET(CTC_OAM_SWDOWNLOAD_BUF_FILEDATA);

            dataLen = inSwDownload.length - 
                CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN - 
                CTC_ORGSPEC_SWDOWNLOAD_FILEDATA_LEN;

            if(dataLen > remainLen)
            {
                /* Insufficient length for parsing */
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO, "[OAM:%s:%d]\n", __FILE__, __LINE__);
                return EPON_OAM_ERR_OK;
            }

            if(dataLen > CTC_ORGSPEC_SWDOWNLOAD_BLOCK_LEN)
            {
                CTC_SWDL_STATE_SET(CTC_OAM_SWDOWNLOAD_BUF_DATAEND);
                ctc_oam_swDownloadError_alloc(inSwDownload, &outSwDownload, CTC_ORGSPEC_SWDL_ERROR_ILLEGALOAM, NULL);
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR, "[OAM:%s:%d]\n", __FILE__, __LINE__);
                break;
            }

            pFp = fopen(CTC_ORGSPEC_SWDL_STORAGE, "ab");
            if(pFp == NULL)
            {
                ctc_oam_swDownloadError_alloc(inSwDownload, &outSwDownload, CTC_ORGSPEC_SWDL_ERROR_MEMORY, NULL);
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR, "[OAM:%s:%d]\n", __FILE__, __LINE__);
                break;
            }
            
            ret = fwrite(pProcPtr, sizeof(char), dataLen, pFp);
            if(ret != dataLen)
            {
                ctc_oam_swDownloadError_alloc(inSwDownload, &outSwDownload, CTC_ORGSPEC_SWDL_ERROR_MEMORY, NULL);
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR, "[OAM:%s:%d]\n", __FILE__, __LINE__);
                fclose(pFp);
                break;
            }
            fclose(pFp);
            CTC_BUF_ADD(pProcPtr, remainLen, dataLen);
            sem_wait(&ctcDataSem);
            ctc_swdl_file.fileSize += dataLen;
            ctc_swdl_file.block ++;
            sem_post(&ctcDataSem);
            if(dataLen != 1400)
            {
                CTC_SWDL_STATE_SET(CTC_OAM_SWDOWNLOAD_BUF_DATAEND);
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
                    "[OAM:%s:%d] File end receive, total size %d, block count %d\n",
                    __FILE__, __LINE__, ctc_swdl_file.fileSize, ctc_swdl_file.block);

				//clear download ack resend timer
				epon_oam_event_send(llidIdx, EPON_OAM_EVENT_DOWNLOAD_ACK_RESEND_CLEAR);
				lastFileData = 1;
            }
            else
            {
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
                    "[OAM:%s:%d] File partial receive, total size %d, block count %d\n",
                    __FILE__, __LINE__, ctc_swdl_file.fileSize, ctc_swdl_file.block);
            }

            /* Ack the block */
            outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_FILE;
            outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_FILEACK;
            outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_FILEACK_LEN;
            outSwDownload.tid = inSwDownload.tid;
            outSwDownload.parse.fileAck.block = ctc_swdl_file.block;
            break;
        case CTC_ORGSPEC_SWDL_OPCODE_FILEACK:
        case CTC_ORGSPEC_SWDL_OPCODE_ERROR:
            /* Only ONU can send this, so there might be some error to receive this */
        default:
            return EPON_OAM_ERR_ORG;
        }
        break;
    case CTC_ORGSPEC_SWDL_TYPE_END:
        switch(inSwDownload.opCode)
        {
        case CTC_ORGSPEC_SWDL_OPCODE_ENDREQ:
            /* Retrive block # */
            if(remainLen < CTC_ORGSPEC_SWDOWNLOAD_ENDREQ_LEN)
            {
                return EPON_OAM_ERR_PARSE;
            }
			//cxy 2016-9-23: olt not send last data pkt of pkt len smaller than 1400 if image size is an integer multiple of 1400
			//				so when recv end request, we think data thransfer compelete.
			if(CTC_SWDL_STATE_GET()==CTC_OAM_SWDOWNLOAD_BUF_FILEDATA)
			{
				EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
	                   "[OAM:%s:%d] set File end by end request\n",__FILE__, __LINE__);
	       		CTC_SWDL_STATE_SET(CTC_OAM_SWDOWNLOAD_BUF_DATAEND);
				epon_oam_event_send(llidIdx, EPON_OAM_EVENT_DOWNLOAD_ACK_RESEND_CLEAR);
			}
			//end of cxy
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE32(&pProcPtr[0], &checkSize);
#else
            ((unsigned char *)&checkSize)[0] = pProcPtr[0];
            ((unsigned char *)&checkSize)[1] = pProcPtr[1];
            ((unsigned char *)&checkSize)[2] = pProcPtr[2];
            ((unsigned char *)&checkSize)[3] = pProcPtr[3];
#endif
            CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_SWDOWNLOAD_ENDREQ_LEN);

            switch(CTC_SWDL_STATE_GET())
            {
            case CTC_OAM_SWDOWNLOAD_BUF_DATAEND:
                sem_wait(&ctcDataSem);
                ctc_swdl_file.checkSize = checkSize;
                sem_post(&ctcDataSem);

                if(ctc_swdl_file.checkSize != ctc_swdl_file.fileSize)
                {
                    outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_END;
                    outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_ENDRESP;
                    outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_ENDRESP_LEN;
                    outSwDownload.tid = inSwDownload.tid;
                    outSwDownload.parse.endResp.respCode = CTC_ORGSPEC_SWDL_RPSCODE_CHECKFAIL;
                    EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
                        "[OAM:%s:%d] File check failed, %d != %d \n", __FILE__, __LINE__, ctc_swdl_file.fileSize, ctc_swdl_file.checkSize);
                    break;
                }
                else
                {
                    EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
                        "[OAM:%s:%d] File check passed, check size %d\n", __FILE__, __LINE__, ctc_swdl_file.checkSize);
                }
                if(bootInfo.active != -1)
                {
                    CTC_SWDL_STATE_SET(CTC_OAM_SWDOWNLOAD_BUF_WRITEING);
                    /* Create thread for software download */
                    ret = pthread_create(&swDownload, NULL, &ctc_oam_swDlThread, NULL);
                    if(ret != 0)
                    {
                        outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_END;
                        outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_ENDRESP;
                        outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_ENDRESP_LEN;
                        outSwDownload.tid = inSwDownload.tid;
                        outSwDownload.parse.endResp.respCode = CTC_ORGSPEC_SWDL_RPSCODE_CHECKFAIL;
                        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
                            "[OAM:%s:%d] download thread start fail %d %d\n", __FILE__, __LINE__, ret, errno);
                        break;
                    }
					gettimeofday(&dowload_start, NULL); /* get download start time */
                }
                else
                {
                    outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_END;
                    outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_ENDRESP;
                    outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_ENDRESP_LEN;
                    outSwDownload.tid = inSwDownload.tid;
                    outSwDownload.parse.endResp.respCode = CTC_ORGSPEC_SWDL_RPSCODE_NOTSUPPORT;
                    EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
                        "[OAM:%s:%d] loader not support dual image\n", __FILE__, __LINE__);
                    break;
                }
                /* Fall through */
            case CTC_OAM_SWDOWNLOAD_BUF_WRITEING:
                /* Programming the flash by the other thread 
                 * Reply and wait for complete
                 */
                outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_END;
                outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_ENDRESP;
                outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_ENDRESP_LEN;
                outSwDownload.tid = inSwDownload.tid;

				gettimeofday(&now, NULL); /* get current time */
				if((now.tv_sec - dowload_start.tv_sec) > CTC_OAM_SWDOWNLOAD_MAX_FAKE_TIME)
				{	/* return download complete reponse if download time exceeds the limit to avoid olt firmware upgrade timeout */
					EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO, "[OAM:%s:%d] download time[%d] exceeds the limit[%d]\n", 
							__FILE__, __LINE__, (now.tv_sec - dowload_start.tv_sec), CTC_OAM_SWDOWNLOAD_MAX_FAKE_TIME);
					outSwDownload.parse.endResp.respCode = CTC_ORGSPEC_SWDL_RPSCODE_COMPLETE;
				}
				else
                	outSwDownload.parse.endResp.respCode = CTC_ORGSPEC_SWDL_RPSCODE_PROGRAMING;
                break;
            case CTC_OAM_SWDOWNLOAD_BUF_WRITEOK:
                outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_END;
                outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_ENDRESP;
                outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_ENDRESP_LEN;
                outSwDownload.tid = inSwDownload.tid;
                outSwDownload.parse.endResp.respCode = CTC_ORGSPEC_SWDL_RPSCODE_COMPLETE;
                break;
            default:
                return EPON_OAM_ERR_ORG;
            }
            break;
        case CTC_ORGSPEC_SWDL_OPCODE_ENDRESP:
            /* Only ONU can send this, so there might be some error to receive this */
        default:
            return EPON_OAM_ERR_ORG;
        }
        break;
    case CTC_ORGSPEC_SWDL_TYPE_ACTIVATE:
        switch(inSwDownload.opCode)
        {
        case CTC_ORGSPEC_SWDL_OPCODE_ACTREQ:
            /* Retrive activate flag */
            if(remainLen < CTC_ORGSPEC_SWDOWNLOAD_ACTREQ_LEN)
            {
                return EPON_OAM_ERR_PARSE;
            }
            activeFlag = pProcPtr[0];
            CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_SWDOWNLOAD_ACTREQ_LEN);

            if(0 != activeFlag)
            {
                outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_ACTIVATE;
                outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_ACTRESP;
                outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_ACTRESP_LEN;
                outSwDownload.tid = inSwDownload.tid;
                outSwDownload.parse.activateResp.ack = CTC_ORGSPEC_SWDL_ACTACK_PARAMETER;
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
                    "[OAM:%s:%d] activate oam parameter incorrect\n", __FILE__, __LINE__);
                break;
            }

            if(CTC_OAM_SWDOWNLOAD_BUF_WRITEOK == CTC_SWDL_STATE_GET())
            {
                if(bootInfo.active != -1)
                {
                	printf("[OAM:%s:%d] ACTREQ sw_active [%d], sw_tryactive to [%d]\n", __FILE__, __LINE__, 
							bootInfo.active,(bootInfo.active == 0) ? 1 : 0);
	
					ret = ctc_oam_swDownloadNv_set("sw_tryactive", (bootInfo.active == 0) ? "1" : "0");
                    if(ret != 0)
                    {
                        outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_ACTIVATE;
                        outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_ACTRESP;
                        outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_ACTRESP_LEN;
                        outSwDownload.tid = inSwDownload.tid;
                        outSwDownload.parse.activateResp.ack = CTC_ORGSPEC_SWDL_ACTACK_FAIL;
                        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
                            "[OAM:%s:%d] active img #%d fail\n", __FILE__, __LINE__, (bootInfo.active == 0) ? 1 : 0);
                        break;
                    }
                }
                else
                {
                    outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_ACTIVATE;
                    outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_ACTRESP;
                    outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_ACTRESP_LEN;
                    outSwDownload.tid = inSwDownload.tid;
                    outSwDownload.parse.activateResp.ack = CTC_ORGSPEC_SWDL_ACTACK_NOTSUPPORT;
                    EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
                        "[OAM:%s:%d] loader not support dual image\n", __FILE__, __LINE__);
                    break;
                }
            }
            outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_ACTIVATE;
            outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_ACTRESP;
            outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_ACTRESP_LEN;
            outSwDownload.tid = inSwDownload.tid;
            outSwDownload.parse.activateResp.ack = CTC_ORGSPEC_SWDL_ACTACK_SUCCESS;
            EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
                "[OAM:%s:%d] activate img #%d\n", __FILE__, __LINE__, (bootInfo.active == 0) ? 1 : 0);

            /* Trigger reboot process */
            ret = pthread_create(&swReboot, NULL, &ctc_oam_rebootThread, NULL);
            if(ret != 0)
            {
                swReboot = 0;
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
                    "[OAM:%s:%d] trigger reboot process fail\n", __FILE__, __LINE__);
                break;
            }
            break;
        case CTC_ORGSPEC_SWDL_OPCODE_ACTRESP:
            /* Only ONU can send this, so there might be some error to receive this */
        default:
            return EPON_OAM_ERR_ORG;
        }
        break;
    case CTC_ORGSPEC_SWDL_TYPE_COMMIT:
        switch(inSwDownload.opCode)
        {
        case CTC_ORGSPEC_SWDL_OPCODE_COMMITREQ:
            /* Retrive commit flag */
            if(remainLen < CTC_ORGSPEC_SWDOWNLOAD_CMTREQ_LEN)
            {
                return EPON_OAM_ERR_PARSE;
            }
            commitFlag = pProcPtr[0];
            CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_SWDOWNLOAD_CMTREQ_LEN);

            if(0 != commitFlag)
            {
                outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_COMMIT;
                outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_COMMITRESP;
                outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_CMTRESP_LEN;
                outSwDownload.tid = inSwDownload.tid;
                outSwDownload.parse.commitResp.ack = CTC_ORGSPEC_SWDL_CMTACK_PARAMETER;
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
                    "[OAM:%s:%d] commit oam parameter incorrect\n", __FILE__, __LINE__);
                break;
            }

            if(bootInfo.commit != -1)
            {
                ret = ctc_oam_swDownloadNv_set("sw_commit", (bootInfo.commit == 0) ? "1" : "0");
                if(ret != 0)
                {
                    outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_COMMIT;
                    outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_COMMITRESP;
                    outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_CMTRESP_LEN;
                    outSwDownload.tid = inSwDownload.tid;
                    outSwDownload.parse.commitResp.ack = CTC_ORGSPEC_SWDL_CMTACK_FAIL;
                    EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
                        "[OAM:%s:%d] commit img #%d fail\n", __FILE__, __LINE__, (bootInfo.commit == 0) ? 1 : 0);
                    break;
                }
            }
            else
            {
                outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_COMMIT;
                outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_COMMITRESP;
                outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_CMTRESP_LEN;
                outSwDownload.tid = inSwDownload.tid;
                outSwDownload.parse.commitResp.ack = CTC_ORGSPEC_SWDL_CMTACK_NOTSUPPORT;
                EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
                    "[OAM:%s:%d] loader not support dual image\n", __FILE__, __LINE__);
                break;
            }
            outSwDownload.dataType = CTC_ORGSPEC_SWDL_TYPE_COMMIT;
            outSwDownload.opCode = CTC_ORGSPEC_SWDL_OPCODE_COMMITRESP;
            outSwDownload.length = CTC_ORGSPEC_SWDOWNLOAD_HDR_LEN + CTC_ORGSPEC_SWDOWNLOAD_CMTRESP_LEN;
            outSwDownload.tid = inSwDownload.tid;
            outSwDownload.parse.commitResp.ack = CTC_ORGSPEC_SWDL_CMTACK_SUCCESS;
            EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
                "[OAM:%s:%d] commit img #%d\n", __FILE__, __LINE__, (bootInfo.commit == 0) ? 1 : 0);

			/* for immediate mode: olt will set commit when onu restart successfully, we need to set commit value */
			if(CTC_OAM_SWDOWNLOAD_BUF_WRITEOK != CTC_SWDL_STATE_GET())
            {
            	printf("[OAM:%s:%d] COMMITREQ bootInfo.commit from [%d] to [%d]\n", __FILE__, __LINE__, 
							bootInfo.commit,(bootInfo.commit == 0) ? 1 : 0);
            	bootInfo.commit = (bootInfo.commit == 0) ? 1 : 0;
			}
            break;
        case CTC_ORGSPEC_SWDL_OPCODE_COMMITRESP:
            /* Only ONU can send this, so there might be some error to receive this */
        default:
            return EPON_OAM_ERR_ORG;
        }
        break;
    default:
        return EPON_OAM_ERR_ORG;
    }
    ctc_oam_orgSpecSwDownload_gen(&outSwDownload, pReplyPtr, &genLen);
    CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
    if(CTC_ORGSPEC_SWDL_OPCODE_ERROR == outSwDownload.opCode)
    {
        ctc_oam_swDownloadError_free(&outSwDownload);
    }
    *pReplyLen = (pReplyPtr - pReplyBuf);

	if(inSwDownload.dataType == CTC_ORGSPEC_SWDL_TYPE_FILE && inSwDownload.opCode == CTC_ORGSPEC_SWDL_OPCODE_FILEDATA
	   && lastFileData == 0)
	{
		return EPON_OAM_ERR_DOWNLOAD_ACK;
	}

    return EPON_OAM_ERR_OK;
}

static int ctc_oam_orgSpecChurning(
    unsigned char llidIdx,      /* LLID index of the incoming operation */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    int ret;
    unsigned char *pProcPtr, *pReplyPtr;
    unsigned short genLen;
    unsigned short remainLen;
    ctc_churning_t inChurning, outChurning;
	int i;
	int keyLen;
	
    /* 1. Parse churning header
     * 2. Process CTC extended OAMPDU according to churning op code
     */
    pProcPtr = pFrame;
    pReplyPtr = pReplyBuf;
    remainLen = length;
    *pReplyLen = 0;

    if(remainLen < CTC_ORGSPEC_CHURNING_HDR_LEN)
    {
        /* Insufficient length for parsing */
        return EPON_OAM_ERR_OK;
    }

    /* Parse common header for churning extended opcode */
    inChurning.churningCode = pProcPtr[0] & CTC_ORGSPEC_CHURNING_CODE_MASK;
    inChurning.keyIdx = pProcPtr[1];
    CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_CHURNING_HDR_LEN);

    switch(inChurning.churningCode)
    {
    case CTC_ORGSPEC_CHURNING_NEWKEYREQ:
        /* TODO - 10G uses different format of new key reply */
        /* Reply the new key by random select one */
#ifdef EPON_OAM_10G_MODE
		keyLen = CTC_ORGSPEC_CHURNING_KEY_10G_LEN;
#else
		keyLen = CTC_ORGSPEC_CHURNING_KEY_LEN;
#endif
		outChurning.churningCode = CTC_ORGSPEC_CHURNING_NEWKEY;
        if(ctc_last_churningKey[llidIdx].keyIdx!=inChurning.keyIdx)
        {
            ctc_oam_churningKey_rand(outChurning.churningKey, keyLen);
			for(i = 0; i < keyLen; i++)
            	ctc_last_churningKey[llidIdx].churningKey[i] = outChurning.churningKey[i];
        }
        else
        {
        	for(i = 0; i < keyLen; i++)
            	outChurning.churningKey[i]=ctc_last_churningKey[llidIdx].churningKey[i];          
        }
        ctc_last_churningKey[llidIdx].keyIdx = inChurning.keyIdx;    

        outChurning.keyIdx = inChurning.keyIdx ^ 0x01; /* Toggle the key index */
        ret = ctc_oam_churningKey_set(llidIdx, outChurning.keyIdx, outChurning.churningKey);
        if(EPON_OAM_ERR_OK != ret ||
          (bufLen < CTC_ORGSPEC_CHURNING_HDR_LEN + keyLen))
        {
            /* Set failed/insufficient length, don't reply to OLT */
            return EPON_OAM_ERR_ORG;
        }

        ctc_oam_orgSpecChurning_gen(&outChurning, pReplyPtr, &genLen, keyLen);
        CTC_BUF_ADD(pReplyPtr, bufLen, genLen);        
        break;
    case CTC_ORGSPEC_CHURNING_NEWKEY:
        /* Only ONU can send this, so there might be some error to receive this */
    default:
        return EPON_OAM_ERR_ORG;
    }
    *pReplyLen = (pReplyPtr - pReplyBuf);

    return EPON_OAM_ERR_OK;
}

static int ctc_oam_orgSpecDba(
    unsigned char llidIdx,      /* LLID index of the incoming operation */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    int ret;
    unsigned char dbaCode;
    unsigned char *pProcPtr, *pReplyPtr;
    unsigned short genLen, parseLen, expectLen;
    unsigned short remainLen;
    ctc_dbaThreshold_t dbaThreshold;

    /* 1. Parse common DBA header
     * 2. Parse DBA header according to different DBA code
     * 3. Process according to different DBA code
     */
    pProcPtr = pFrame;
    pReplyPtr = pReplyBuf;
    remainLen = length;
    *pReplyLen = 0;
    expectLen = 0;

    if(remainLen < CTC_ORGSPEC_DBA_HDR_LEN)
    {
        /* Insufficient length for parsing */
        return EPON_OAM_ERR_OK;
    }

    /* Parse DBA common header */
    dbaCode = pProcPtr[0] & CTC_ORGSPEC_DBA_CODE_MASK;
    CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_DBA_HDR_LEN);
    switch(dbaCode)
    {
    case CTC_ORGSPEC_DBA_GET_REQ:
        /* Get DBA threshold from database */
        ctc_oam_dbaConfig_get(&dbaThreshold);

        /* Check reply buffer length */
        expectLen = ctc_oam_orgSpecDba_calc(
            CTC_ORGSPEC_DBA_GET_RESP, &dbaThreshold);
        if(bufLen < expectLen)
        {
            /* Insufficient length for reply */
            break;
        }

        /* Generate DBA reply */
        ctc_oam_orgSpecDba_gen(
            CTC_ORGSPEC_DBA_GET_RESP,
            &dbaThreshold,
            0,
            pReplyPtr,
            &genLen);
        CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
        break;
    case CTC_ORGSPEC_DBA_SET_REQ:
        /* Length calculation in parse function */
        ctc_oam_orgSpecDbaTheshold_parse(pProcPtr, remainLen, &parseLen, &dbaThreshold);
        if(0 == parseLen)
        {
            /* Incorrect frame format */
            break;
        }
        CTC_BUF_ADD(pProcPtr, remainLen, parseLen);

        /* Check reply buffer length */
        expectLen = ctc_oam_orgSpecDba_calc(
            CTC_ORGSPEC_DBA_SET_RESP, &dbaThreshold);
        if(bufLen < expectLen)
        {
            /* Insufficient length for reply */
            break;
        }

        ret = ctc_oam_dbaConfig_set(&dbaThreshold);
        /* Use set result to indicate the set ACK */
        ctc_oam_orgSpecDba_gen(
            CTC_ORGSPEC_DBA_SET_RESP,
            &dbaThreshold,
            (ret == EPON_OAM_ERR_OK) ? 0x01 : 0x00,
            pReplyPtr,
            &genLen);
        CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
        break;
    case CTC_ORGSPEC_DBA_GET_RESP:
    case CTC_ORGSPEC_DBA_SET_RESP:
        /* According to CTC standard, only ONU can send this to OLT
         * ONU should not receive these two, just ignore it
         */
    default:
        /* Impossible case */
        return EPON_OAM_ERR_OK;
    }
    *pReplyLen = (pReplyPtr - pReplyBuf);

    return EPON_OAM_ERR_OK;
}

static void ctc_oam_orgSpecEvent_status_set(ctc_eventOam_t *eventStatus)
{
    int enable;
    
    if(eventStatus->eventStatus == CTC_ORGSPEC_EVENT_EVENTSTATUS_DISABLE)
    {
        enable = DISABLED;
    }
    else if(eventStatus->eventStatus == CTC_ORGSPEC_EVENT_EVENTSTATUS_ENABLE)
    {
        enable = ENABLED;
    }
    else
    {
        /*ignore not valid value*/
        return;
    }
    ctc_oam_alarm_state_set(eventStatus->alarmId, eventStatus->objectInstance,enable);
}

static void ctc_oam_orgSpecEvent_threshold_set(ctc_thresholdOam_t *thresholdStatus)
{
    ctc_oam_alarm_threshold_set(thresholdStatus->alarmId, thresholdStatus->objectInstance, 
                                thresholdStatus->setThreshold,thresholdStatus->clearThreshold);
}

static int ctc_oam_orgSpecEvent(
    unsigned char llidIdx,      /* LLID index of the incoming operation */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    int ret;
    unsigned char eventCode;
    unsigned char *pProcPtr, *pReplyPtr;
    unsigned short genLen, parseLen, expectLen;
    unsigned short remainLen;
	unsigned short entryCount;
	ctc_eventOam_t  eventStatus[CTC_EVENT_COUNT_MAX] = {{0}};
    ctc_thresholdOam_t thresholdStatus[CTC_EVENT_COUNT_MAX] = {{0}};
    int i;
	int offset = 0;

    /* 1. Parse common event header
     * 2. Parse event header according to different subtype
     * 3. Process according to different subtype
     */
    pProcPtr = pFrame;
    pReplyPtr = pReplyBuf;
    remainLen = length;
    *pReplyLen = 0;
    expectLen = 0;

    if(remainLen < CTC_ORGSPEC_EVENT_HDR_LEN)
    {
        /* Insufficient length for parsing */
        return EPON_OAM_ERR_OK;
    }

	/* Parse event common header */
    eventCode = pProcPtr[0];
#ifdef SUPPORT_OAM_BYTEORDER
	CTC_BUF_PARSE16(&pProcPtr[1], &entryCount);
#else
    ((unsigned char *)&entryCount)[0] = pProcPtr[1];
    ((unsigned char *)&entryCount)[1] = pProcPtr[2]; 
#endif

	EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO, 
        "[OAM:%s:%d] event subtype[%d] entryCount[%d]\n", 
        __FILE__, __LINE__, eventCode, entryCount);
	
    entryCount = entryCount > CTC_EVENT_COUNT_MAX ? CTC_EVENT_COUNT_MAX : entryCount;

    CTC_BUF_ADD(pProcPtr, remainLen, CTC_ORGSPEC_EVENT_HDR_LEN);
    memset(eventStatus,0,sizeof(ctc_eventOam_t));
    memset(thresholdStatus,0,sizeof(ctc_thresholdOam_t));
	 
    switch(eventCode)
    {
    case CTC_ORGSPEC_EVENT_STATUS_REQ:
        for(i = 0; i < entryCount; i++ )
        {
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE16(&pProcPtr[offset], &eventStatus[i].objectType);
#else
            memcpy(&(eventStatus[i].objectType), &pProcPtr[offset], 2);
#endif
            offset+=2;
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE32(&pProcPtr[offset], &eventStatus[i].objectInstance);
#else
            memcpy(&(eventStatus[i].objectInstance), &pProcPtr[offset], 4);
#endif
            offset+=4;
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE16(&pProcPtr[offset], &eventStatus[i].alarmId);
#else
            memcpy(&(eventStatus[i].alarmId), &pProcPtr[offset], 2);
#endif
            offset+=2;
        }
        /*Generate Event reply*/
        ctc_oam_orgSpecEvent_gen(
                CTC_ORGSPEC_EVENT_STATUS_RESP,
                eventStatus,
                thresholdStatus,
                entryCount,
                pReplyPtr,
                &genLen);
        CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
        break;
    case CTC_ORGSPEC_EVENT_STATUS_SET:
	    for(i = 0; i < entryCount; i++ )
        {
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE16(&pProcPtr[offset], &eventStatus[i].objectType);
#else
            memcpy(&(eventStatus[i].objectType), &pProcPtr[offset], 2);
#endif
            offset += 2;
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE32(&pProcPtr[offset], &eventStatus[i].objectInstance);
#else
            memcpy(&(eventStatus[i].objectInstance), &pProcPtr[offset], 4);
#endif
            offset += 4;
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE16(&pProcPtr[offset], &eventStatus[i].alarmId);
#else
            memcpy(&(eventStatus[i].alarmId), &pProcPtr[offset], 2);
#endif
            offset += 2;
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE32(&pProcPtr[offset], &eventStatus[i].eventStatus);
#else
            memcpy(&(eventStatus[i].eventStatus), &pProcPtr[offset], 4);
#endif
            offset += 4;
            ctc_oam_orgSpecEvent_status_set(&eventStatus[i]);
        }
        ctc_oam_orgSpecEvent_gen(
                CTC_ORGSPEC_EVENT_STATUS_RESP,
                eventStatus,
                thresholdStatus,
                entryCount,
                pReplyPtr,
                &genLen);
        CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
        break;
    case CTC_ORGSPEC_EVENT_THRESHOLD_REQ:
        for(i = 0; i < entryCount; i++ )
        {
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE16(&pProcPtr[offset], &thresholdStatus[i].objectType);
#else
            memcpy(&(thresholdStatus[i].objectType), &pProcPtr[offset], 2);
#endif
            offset += 2;
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE32(&pProcPtr[offset], &thresholdStatus[i].objectInstance);
#else
            memcpy(&(thresholdStatus[i].objectInstance), &pProcPtr[offset], 4);
#endif
            offset += 4;
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE16(&pProcPtr[offset], &thresholdStatus[i].alarmId);
#else
            memcpy(&(thresholdStatus[i].alarmId), &pProcPtr[offset], 2);
#endif
            offset += 2;
        }
        /*Generate Event reply*/
        ctc_oam_orgSpecEvent_gen(
                CTC_ORGSPEC_EVENT_THRESHOLD_RESP,
                eventStatus,
                thresholdStatus,
                entryCount,
                pReplyPtr,
                &genLen);
        CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
        break;
    case CTC_ORGSPEC_EVENT_THRESHOLD_SET:
        for(i = 0; i < entryCount; i++ )
        {
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE16(&pProcPtr[offset], &thresholdStatus[i].objectType);
#else
            memcpy(&(thresholdStatus[i].objectType), &pProcPtr[offset], 2);
#endif
            offset += 2;
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE32(&pProcPtr[offset], &thresholdStatus[i].objectInstance);
#else
            memcpy(&(thresholdStatus[i].objectInstance), &pProcPtr[offset], 4);
#endif
            offset += 4;
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE16(&pProcPtr[offset], &thresholdStatus[i].alarmId);
#else
            memcpy(&(thresholdStatus[i].alarmId), &pProcPtr[offset], 2);
#endif
            offset += 2;
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE32(&pProcPtr[offset], &thresholdStatus[i].setThreshold);
#else
            memcpy(&(thresholdStatus[i].setThreshold), &pProcPtr[offset], 4);
#endif
            offset += 4;
#ifdef SUPPORT_OAM_BYTEORDER
			CTC_BUF_PARSE32(&pProcPtr[offset], &thresholdStatus[i].clearThreshold);
#else
            memcpy(&(thresholdStatus[i].clearThreshold), &pProcPtr[offset], 4);
#endif
            offset += 4;
            ctc_oam_orgSpecEvent_threshold_set(&thresholdStatus[i]);
        }

        /*Generate Event reply*/
        ctc_oam_orgSpecEvent_gen(
                    CTC_ORGSPEC_EVENT_THRESHOLD_RESP,
                    eventStatus,
                    thresholdStatus,
                    entryCount,
                    pReplyPtr,
                    &genLen);
        CTC_BUF_ADD(pReplyPtr, bufLen, genLen);
        break;
    case CTC_ORGSPEC_EVENT_STATUS_RESP:
    case CTC_ORGSPEC_EVENT_THRESHOLD_RESP:
        /* According to CTC standard, only ONU can send this to OLT
         * ONU should not receive these two, just ignore it
         */
    default:
        /* Impossible case */
        return EPON_OAM_ERR_OK;
    }
    *pReplyLen = (pReplyPtr - pReplyBuf);

    return EPON_OAM_ERR_OK;
}

static int ctc_oam_extInfoStd_parser(
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    ctc_infoOam_t *pInfoOam
)
{
    unsigned char verListLen;
    unsigned char *pParsePtr;
    ctc_infoOamVerRec_t *pVerListNew, *pVerList;
    
    pInfoOam->vertionList = NULL;
    pParsePtr = pFrame;
    pInfoOam->type = *pParsePtr;
    pParsePtr += 1; /* Type */
    pInfoOam->length = *pParsePtr;
    pParsePtr += 1; /* Length */

    /* Length check */
    if(length < pInfoOam->length)
    {
        /* Incorrect length field */
        return 0;
    }

    /* Version list length correction */
    verListLen = (pInfoOam->length - CTC_INFO_OAM_MIN);
    if((verListLen % CTC_INFO_OAM_VERITEM_LEN) != 0) {
        /* version list should be integer times of CTC_INFO_OAM_VERITEM_LEN */
        verListLen -= (verListLen % CTC_INFO_OAM_VERITEM_LEN);
    }

    memcpy(pInfoOam->oui, pParsePtr, 3);
    pParsePtr += 3; /* OUI */
    pInfoOam->extSupport = *pParsePtr;
    pParsePtr += 1; /* ExtSupport */

	/* ExtSupport check, if ExtSupport is equal to 1 it means support extended OAM, 0
	 * means not support extended OAM, any other value should be ignore.
	 */
	if ((0 != pInfoOam->extSupport) && (1 != pInfoOam->extSupport))
		return 0;
	
    pInfoOam->version = *pParsePtr;
    pParsePtr += 1; /* Version */

    /* Parse version list */
    while(0 != verListLen)
    {
        ctc_oam_verListItem_alloc(pParsePtr, *(pParsePtr + 3), &pVerListNew);
        if(NULL == pVerListNew)
        {
            /* Allocate failed */
            /* Just ignore the remain length */
            break;
        }

        if(NULL == pInfoOam->vertionList)
        {
            pInfoOam->vertionList = pVerListNew;
        }
		else
		{
			pVerList->next = pVerListNew;
		}
        pVerList = pVerListNew;

        verListLen -= CTC_INFO_OAM_VERITEM_LEN;
        pParsePtr += CTC_INFO_OAM_VERITEM_LEN;
    }

    return pInfoOam->length;
}

static int ctc_oam_extInfo_parser(
    unsigned char llidIdx,      /* LLID index from HW table */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned short *pExtractLen,/* Parser extract length */
    void **ppOrgSpecData)       /* Orgnization specific data */
{
    ctc_infoOam_t *pInfoOam;

    /* Check for minimum length for parsing */
    if(length < CTC_INFO_OAM_MIN)
    {
        return EPON_OAM_ERR_ORG;
    }

    pInfoOam = (ctc_infoOam_t *) malloc(sizeof(ctc_infoOam_t));
    if(NULL == pInfoOam)
    {
        return EPON_OAM_ERR_ORG;
    }

    /* Confirm length before parsing */
    *pExtractLen = ctc_oam_extInfoStd_parser(pFrame, length, pInfoOam);
    if(0 == *pExtractLen)
    {
        if(NULL != pInfoOam->vertionList)
        {
            ctc_oam_verListItem_free(&pInfoOam->vertionList);
        }
        free(pInfoOam);
        *ppOrgSpecData = NULL;
        return EPON_OAM_ERR_ORG;
    }
    *ppOrgSpecData = pInfoOam;

    return EPON_OAM_ERR_OK;
}

/* MODIFY ME - need further enhance for exception cases in all states */
static int ctc_oam_extInfo_handler(
    oam_oamPdu_t *pOamPdu,      /* OAMPDU data */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen,  /* Reply size used by this handler*/
    void **ppOrgSpecData)       /* Orgnization specific data
                                 * Direct comes from parser
                                 */
{
    int ret = EPON_OAM_ERR_OK;
    unsigned short expectLen;
    unsigned char extSupport;
    ctc_infoOam_t *pInfoOam;

    if(NULL != ppOrgSpecData)
    {
        pInfoOam = *ppOrgSpecData;
    }
    else
    {
        return EPON_OAM_ERR_PARAM;
    }
    if(NULL == pInfoOam)
    {
        return EPON_OAM_ERR_PARAM;
    }

    dump_ctc_infoOam(pInfoOam);


    EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO, 
        "[OAM:%s:%d] ctc_discovery_state[LLIDidx:%d]=%d \n", __FILE__, __LINE__, pOamPdu->llidIdx,ctc_discovery_state[pOamPdu->llidIdx]);


    switch(ctc_discovery_state[pOamPdu->llidIdx])
    {
    case CTC_OAM_FSM_STATE_WAIT_REMOTE:
        ctc_discovery_state[pOamPdu->llidIdx] = CTC_OAM_FSM_STATE_WAIT_REMOTE_OK;
        extSupport = ctc_oam_version_support(pInfoOam->oui, pInfoOam->version);
        expectLen = CTC_INFO_OAM_MIN + sizeof(supportedVerList);
        if(bufLen < expectLen)
        {
            *pReplyLen = 0;
            ret = EPON_OAM_ERR_ORG;
        }
        else
        {
            ctc_oam_Info_gen(expectLen, extSupport, 0x00, pInfoOam, 1, pReplyBuf);
            *pReplyLen = expectLen;
        }
        break;
    case CTC_OAM_FSM_STATE_WAIT_REMOTE_OK:
        /* Check if the OUI/version pair in reply is supported */
        extSupport = ctc_oam_version_support(pInfoOam->oui, pInfoOam->version);
        if((1 == extSupport) && (1 == pInfoOam->extSupport))
        {
            /* Both ONU and OLT accept the OUI/Version pair */
            ctc_discovery_state[pOamPdu->llidIdx] = CTC_OAM_FSM_STATE_COMPLETE;
            memcpy(currCtcVer[pOamPdu->llidIdx].oui, pInfoOam->oui, EPON_OAM_OUI_LENGTH);
            currCtcVer[pOamPdu->llidIdx].version = pInfoOam->version;
        }
        else
        {
            /* At least one side didn't satisfied the OUI/Version */
            ctc_discovery_state[pOamPdu->llidIdx] = CTC_OAM_FSM_STATE_WAIT_REMOTE;
            /* Force send out NACK */
            extSupport = 0;
        }

        expectLen = CTC_INFO_OAM_MIN;
        if(bufLen < expectLen)
        {
            *pReplyLen = 0;
            ret = EPON_OAM_ERR_ORG;
        }
        else
        {
            ctc_oam_Info_gen(expectLen, extSupport, pInfoOam->version, pInfoOam, 0, pReplyBuf);
            *pReplyLen = expectLen;
        }
        break;
    case CTC_OAM_FSM_STATE_COMPLETE:
        /* Ignore any extended info after extended discovery compelete */
        if(pInfoOam->vertionList!=NULL&&pInfoOam->extSupport==1)
        {
         ctc_discovery_state[pOamPdu->llidIdx] = CTC_OAM_FSM_STATE_WAIT_REMOTE;
        }
        ret = EPON_OAM_ERR_OK;
        break;
    default:
        /* Not supported states */
        break;
    }

    ctc_oam_verListItem_free(&pInfoOam->vertionList);
    free(pInfoOam);
    
    return ret;
}

static int
ctc_oam_orgSpec_processor(
    oam_oamPdu_t *pOamPdu,      /* OAMPDU data */
    unsigned char *pFrame,      /* Frame payload current pointer */
    unsigned short length,      /* Frame payload length */
    unsigned char *pReplyBuf,   /* Frame buffer for reply OAM */
    unsigned short bufLen,      /* Frame buffer size */
    unsigned short *pReplyLen)  /* Reply size used by this handler*/
{
    int ret = EPON_OAM_ERR_OK;
    unsigned char oui[3];
    unsigned char currVer;
    unsigned char extOpcode;
    unsigned short genLen;

    /* 0. Check for extended discovery state
     * 1. Parsing the frame
     * 2. Process frame according to the extended opcode
     * 3. Reply extended OAMPDU
     */
    *pReplyLen = 0;
    if((length < CTC_ORGSPEC_HDR_LEN) || (bufLen < CTC_ORGSPEC_HDR_LEN))
    {
        /* Insufficient length for parsing */
        return EPON_OAM_ERR_PARAM;
    }

    /* Parse the extended OAM header */
    oui[0] = pFrame[0];
    oui[1] = pFrame[1];
    oui[2] = pFrame[2];
    extOpcode = pFrame[3];
    CTC_BUF_ADD(pFrame, length, CTC_ORGSPEC_HDR_LEN);
    EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO, 
        "[OAM:%s:%d] ext. opcode %d\n", __FILE__, __LINE__, extOpcode);

    if(CTC_OAM_FSM_STATE_COMPLETE != ctc_discovery_state[pOamPdu->llidIdx])
    {
        /* Extended OAM discovery not complete
         * Don't know what version to be used
         * Try to use CTC 1.0
         */
        currVer = 0x01;
    }
    else
    {
        currVer = currCtcVer[pOamPdu->llidIdx].version;
    }   

    switch(extOpcode)
    {
    case CTC_EXTOAM_OPCODE_VARREQ:
        ctc_oam_orgSpecHdr_gen(oui, CTC_EXTOAM_OPCODE_VARRESP, pReplyBuf, &genLen);
        CTC_BUF_ADD(pReplyBuf, bufLen, genLen);
        *pReplyLen += genLen;
        genLen= 0;
#if 0
        if(currVer <= CTC_OAM_VERSION_20)
        {
#if CTC_OAM_SUPPORT_BENEATH_20
            ret = ctc_oam_orgSpecVar_req20(
                pOamPdu->llidIdx, pFrame, length, pReplyBuf, bufLen, &genLen);
#endif /* CTC_OAM_SUPPORT_BENEATH_20 */
        }
        else
        {
#if CTC_OAM_SUPPORT_ABOVE_21
            ret = ctc_oam_orgSpecVar_req21(
                pOamPdu->llidIdx, pFrame, length, pReplyBuf, bufLen, &genLen);
#endif /* CTC_OAM_SUPPORT_ABOVE_21 */
        }
#else
        ret = ctc_oam_orgSpecVar_req(
            pOamPdu->llidIdx, pFrame, length, pReplyBuf, bufLen, &genLen);
#endif
        if(0 != genLen)
        {
            *pReplyLen += genLen;
        }
        else
        {
            /* Nothing to be replied, remove encoded header */
            *pReplyLen = 0;
        }
        break;
    case CTC_EXTOAM_OPCODE_VARRESP:
        /* Should not receive this on ONU */
        break;
    case CTC_EXTOAM_OPCODE_SETREQ:
        ctc_oam_orgSpecHdr_gen(oui, CTC_EXTOAM_OPCODE_SETRESP, pReplyBuf, &genLen);
        CTC_BUF_ADD(pReplyBuf, bufLen, genLen);
        *pReplyLen += genLen;
        genLen= 0;
#if 0
        if(currVer <= CTC_OAM_VERSION_20)
        {
#if CTC_OAM_SUPPORT_BENEATH_20
            ret = ctc_oam_orgSpecSet_req20(
                pOamPdu->llidIdx, pFrame, length, pReplyBuf, bufLen, &genLen);
#endif /* CTC_OAM_SUPPORT_BENEATH_20 */
        }
        else
        {
#if CTC_OAM_SUPPORT_ABOVE_21
            ret = ctc_oam_orgSpecSet_req21(
                pOamPdu->llidIdx, pFrame, length, pReplyBuf, bufLen, &genLen);
#endif /* CTC_OAM_SUPPORT_ABOVE_21 */
        }
#else
        ret = ctc_oam_orgSpecSet_req(
            pOamPdu->llidIdx, pFrame, length, pReplyBuf, bufLen, &genLen);
#endif
        if(0 != genLen)
        {
            *pReplyLen += genLen;
        }
        else
        {
            /* Nothing to be replied, remove encoded header */
            *pReplyLen = 0;
        }
        break;
    case CTC_EXTOAM_OPCODE_SETRESP:
        /* Should not receive this on ONU */
        break;
    case CTC_EXTOAM_OPCODE_ONUAUTH:
        ctc_oam_orgSpecHdr_gen(oui, CTC_EXTOAM_OPCODE_ONUAUTH, pReplyBuf, &genLen);
        CTC_BUF_ADD(pReplyBuf, bufLen, genLen);
        *pReplyLen += genLen;
        genLen= 0;
        ret = ctc_oam_orgSpecOnuAuth(
            pOamPdu->llidIdx, pFrame, length, pReplyBuf, bufLen, &genLen);
        if(0 != genLen)
        {
            *pReplyLen += genLen;
        }
        else
        {
            /* Nothing to be replied, remove encoded header */
            *pReplyLen = 0;
        }
        break;
    case CTC_EXTOAM_OPCODE_SWDOWNLOAD:
        ctc_oam_orgSpecHdr_gen(oui, CTC_EXTOAM_OPCODE_SWDOWNLOAD, pReplyBuf, &genLen);
        CTC_BUF_ADD(pReplyBuf, bufLen, genLen);
        *pReplyLen += genLen;
        genLen= 0;
        ret = ctc_oam_orgSpecSwDownload(
            pOamPdu->llidIdx, pFrame, length, pReplyBuf, bufLen, &genLen);
        if(0 != genLen)
        {
            *pReplyLen += genLen;

			//save download ack packet content
			if(ret == EPON_OAM_ERR_DOWNLOAD_ACK)
			{
				unsigned char * ptr;

				EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_INFO,
                    "[OAM:%s:%d] download ack size[%d]\n",
                    __FILE__, __LINE__,*pReplyLen);
				ptr = pReplyBuf - (*pReplyLen - genLen);
				epon_oam_eventData_send(pOamPdu->llidIdx, EPON_OAM_EVENT_DOWNLOAD_ACK_RESEND_TIMER, ptr, *pReplyLen);
				ret = EPON_OAM_ERR_OK;
			}
        }
        else
        {
            /* Nothing to be replied, remove encoded header */
            *pReplyLen = 0;
        }
        break;
    case CTC_EXTOAM_OPCODE_CHURNING:
        ctc_oam_orgSpecHdr_gen(oui, CTC_EXTOAM_OPCODE_CHURNING, pReplyBuf, &genLen);
        CTC_BUF_ADD(pReplyBuf, bufLen, genLen);
        *pReplyLen += genLen;
        genLen= 0;
        ret = ctc_oam_orgSpecChurning(
            pOamPdu->llidIdx, pFrame, length, pReplyBuf, bufLen, &genLen);
        if(0 != genLen)
        {
            *pReplyLen += genLen;
        }
        else
        {
            /* Nothing to be replied, remove encoded header */
            *pReplyLen = 0;
        }
        break;
    case CTC_EXTOAM_OPCODE_DBA:
        ctc_oam_orgSpecHdr_gen(oui, CTC_EXTOAM_OPCODE_DBA, pReplyBuf, &genLen);
        CTC_BUF_ADD(pReplyBuf, bufLen, genLen);
        *pReplyLen += genLen;
        genLen= 0;
        ret = ctc_oam_orgSpecDba(
            pOamPdu->llidIdx, pFrame, length, pReplyBuf, bufLen, &genLen);
        if(0 != genLen)
        {
            *pReplyLen += genLen;
        }
        else
        {
            /* Nothing to be replied, remove encoded header */
            *pReplyLen = 0;
        }
        break;
    case CTC_EXTOAM_OPCODE_EVENT:
        ctc_oam_orgSpecHdr_gen(oui, CTC_EXTOAM_OPCODE_EVENT, pReplyBuf, &genLen);
        CTC_BUF_ADD(pReplyBuf, bufLen, genLen);
        *pReplyLen += genLen;
        genLen= 0;
        ret = ctc_oam_orgSpecEvent(
            pOamPdu->llidIdx, pFrame, length, pReplyBuf, bufLen, &genLen);
        if(0 != genLen)
        {
            *pReplyLen += genLen;
        }
        else
        {
            /* Nothing to be replied, remove encoded header */
            *pReplyLen = 0;
        }
        break;
    case CTC_EXTOAM_OPCODE_RESERVED:
    default:
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_WARN,
            "[OAM:%s:%d] illegal ext opcode %u\n", __FILE__, __LINE__, extOpcode);
        break;
    }

    if(EPON_OAM_ERR_OK != ret)
    {
        /* Something wrong, remove all reply */
        *pReplyLen = 0;
    }

    return ret;
}

int ctc_oam_evtNotification_send(
    ctc_eventOam_t *pEvent)
{
    
    return EPON_OAM_ERR_OK;
}

int ctc_oam_flash_var_get(char * mibName, unsigned char *var, unsigned int size)
{
    unsigned char buf[128];
	unsigned char cmd[128] = {0};
    unsigned char *flashbuf = NULL;
	FILE *pFD;

    memset(buf, 0, 128);

	snprintf(cmd, 128, "/etc/scripts/flash get %s", mibName);

	pFD = popen(cmd, "r");
    if (pFD)
    {
        if (fgets(buf, 128, pFD))
        {
            strtok(buf, "=");
            flashbuf = strtok(NULL, "");
        }
        pclose(pFD);
    }
    
    if (flashbuf)
    {
        int i;
        /* remove line break */
        for (i=0; i<size; i++)
        {
            if ((flashbuf[i]=='\n') || (flashbuf[i]=='\r'))
            {
                flashbuf[i] = '\0';
                break;
            }
        }
        memcpy(var, flashbuf, size);

        return EPON_OAM_ERR_OK;
    }

    return EPON_OAM_ERR_NOT_FOUND;

}

/* var type is byte array, need to convert byte value to string value 
 * byte value exmaple (hex format): "52,54,4B,4C", convert the byte value to string vlaue "RTKL"
 */
int ctc_oam_flash_byte_var_get(char * mibName, unsigned char *var, unsigned int size)
{
    unsigned char buf[128];
	unsigned char cmd[128] = {0};
    unsigned char *flashbuf = NULL;
	FILE *pFD;

    memset(buf, 0, 128);

	snprintf(cmd, 128, "/etc/scripts/flash get %s", mibName);

	pFD = popen(cmd, "r");
    if (pFD)
    {
        if (fgets(buf, 128, pFD))
        {
            strtok(buf, "=");
            flashbuf = strtok(NULL, "");
        }
        pclose(pFD);
    }
    
    if (flashbuf)
    {
        int i;
		int offset = 0;
		int tmp, ret;

        for (i=0; i<size; i++)
        {
        	flashbuf[offset+2] = '\0';
			ret = sscanf(flashbuf+offset, "%x", &tmp);
			if(ret <= 0) /* sscanf not found or parse error */
				break;
			var[i] = tmp;
			offset += 3;
			//printf("%d[%d:0x%x]\n",i,offset-3,var[i]);
        }
        return EPON_OAM_ERR_OK;
    }

    return EPON_OAM_ERR_NOT_FOUND;
}


void ctc_oam_flash_var_set(char * mibName, unsigned char *value)
{
	unsigned char cmdStr[256] = {0};

	snprintf(cmdStr, sizeof(cmdStr), "/etc/scripts/flash set %s %s", mibName, value);
    system(cmdStr);
}

int ctc_oam_env_get(char *env, unsigned char *var, unsigned int size)
{
    unsigned char buf[128];
	unsigned char cmd[128] = {0};
    unsigned char *flashbuf = NULL;
	FILE *pFD;

    memset(buf, 0, 128);

	snprintf(cmd, 128, "/bin/nv getenv %s", env);

	pFD = popen(cmd, "r");
    if (pFD)
    {
        if (fgets(buf, 128, pFD))
        {
            strtok(buf, "=");
            flashbuf = strtok(NULL, "");
        }
        pclose(pFD);
    }

    if (flashbuf)
    {
        int i;
        /* remove line break */
        for (i=0; i<size; i++)
        {
            if ((flashbuf[i]=='\n') || (flashbuf[i]=='\r'))
            {
                flashbuf[i] = '\0';
                break;
            }
        }
        memcpy(var, flashbuf, size);

        return EPON_OAM_ERR_OK;
    }

    return EPON_OAM_ERR_NOT_FOUND;
}

void ctc_oam_port_remapping_init()
{
    unsigned char mapStr[20];
    unsigned int mapTbl[4];
    int ret;

    ret = ctc_oam_flash_var_get(CTC_OAM_PORT_REMAPPING, mapStr, sizeof(mapStr));
    if (EPON_OAM_ERR_OK == ret)
    {
        sscanf(mapStr, "%2x,%2x,%2x,%2x", &mapTbl[0], &mapTbl[1], &mapTbl[2], &mapTbl[3]);
        set_port_remapping_table(mapTbl, 4);
    }
}

static void ctc_oam_sn_init()
{
	int ret;
	unsigned char buff[64];
	int len,offset;
	memset(&ctcOnuSn, 0, sizeof(ctc_onuSnInfo_t));
	
	ret = ctc_oam_flash_var_get(CTC_OAM_VENDOR_ID, ctcOnuSn.vendorID, sizeof(ctcOnuSn.vendorID));
	if(ret != EPON_OAM_ERR_OK)
	{
		sprintf(ctcOnuSn.vendorID, "%s", "RTKG");
	}
#ifdef YUEME_CUSTOMIZED_CHANGE
	memset(buff, 0, sizeof(buff));
	ret = ctc_oam_flash_var_get(CTC_OAM_ONU_MODEL, buff, sizeof(buff));
	if(ret == EPON_OAM_ERR_OK)
	{
		len = strlen(buff);
		offset = (len>4) ? (len-4) : 0;
		memcpy(ctcOnuSn.onuModel, buff+offset, (len<4) ? len : sizeof(ctcOnuSn.onuModel) );
		//printf("%s len[%d] buf[%s], model[%s]\n", __func__, len, buff, buff+offset);
	}
	else
	{
		sprintf(ctcOnuSn.onuModel, "%s", "RTKG");
	}
#else
    ret = ctc_oam_flash_var_get(CTC_OAM_ONU_MODEL, ctcOnuSn.onuModel, sizeof(ctcOnuSn.onuModel));
	if(ret != EPON_OAM_ERR_OK)
	{
		sprintf(ctcOnuSn.onuModel, "%s", "RTKG");
	}
#endif
    ret = ctc_oam_flash_var_get(CTC_OAM_HW_VERSION, ctcOnuSn.hwVersion, sizeof(ctcOnuSn.hwVersion));
	if(ret != EPON_OAM_ERR_OK)
	{
		sprintf(ctcOnuSn.hwVersion, "%s", "RTL960x");
	}
	//ctc_oam_flash_var_get(CTC_OAM_SW_VERSION, ctcOnuSn.swVersion, sizeof(ctcOnuSn.swVersion));
	{
		int swActive=0;
		char str_active[64], envName[64];

        ctc_oam_env_get("sw_active", str_active, sizeof(str_active));
		sprintf(envName, "sw_version%s", str_active);
        ctc_oam_env_get(envName, ctcOnuSn.swVersion, sizeof(ctcOnuSn.swVersion));
	}
#ifdef YUEME_CUSTOMIZED_CHANGE
	memset(ctcOnuSn.extOnuModel, 0, sizeof(ctcOnuSn.extOnuModel));
#else
    ret = ctc_oam_flash_var_get(CTC_OAM_EXTONU_MODEL, ctcOnuSn.extOnuModel, sizeof(ctcOnuSn.extOnuModel));
	if(ret != EPON_OAM_ERR_OK)
	{
		sprintf(ctcOnuSn.extOnuModel, "%s", "RTKG");
	}
#endif

}

#ifdef YUEME_CUSTOMIZED_CHANGE
/* auth type value in mib entry
	0 : MAC mode
	1 : LOID mode
	2 : MAC+LOID mode
*/
static void ctc_oam_defaultAuthType_init()
{
    unsigned char authTypeStr[4];
    unsigned int authType;
    int ret;

    ret = ctc_oam_flash_var_get(CTC_OAM_PON_REG_MODE, authTypeStr, sizeof(authTypeStr));
    if (EPON_OAM_ERR_OK == ret)
    {
        authType = atoi(authTypeStr);
		if(authType == 0)
			epon_oam_defaultAuthType_set(0, CTC_OAM_ONUAUTH_AUTH_MAC);
		else if(authType == 1)
			epon_oam_defaultAuthType_set(0, CTC_OAM_ONUAUTH_AUTH_LOID);
		else if(authType == 2)
			epon_oam_defaultAuthType_set(0, CTC_OAM_ONUAUTH_AUTH_MAC_LOID);
		else
			epon_oam_defaultAuthType_set(0, CTC_OAM_ONUAUTH_AUTH_MAC);
    }
	else
		epon_oam_defaultAuthType_set(0, CTC_OAM_ONUAUTH_AUTH_MAC);
}
#endif

static void ctc_oam_silent_mode_init()
{
	unsigned int enable = 0;
	char tmp[4];

	if (EPON_OAM_ERR_OK == ctc_oam_flash_var_get(CTC_OAM_SILENT_MODE, tmp, sizeof(tmp))) 
	{
		enable = atoi(tmp);
	}
	epon_ctc_silent_mode_set(enable);
	epon_ctc_silent_pon_led_mode_set(0);
}

/* init for 0xC7/0x00A1 - ONUTxPowerSupplyControl */
int ctc_oam_gpio_tx_disable_pin_get(unsigned int * gpio_disTx)
{
	*gpio_disTx = gpio_disTx_pin;
	return EPON_OAM_ERR_OK;
}

static void ctc_oam_gpio_tx_disable_init()
{
	unsigned int gpio_disTx;
	int ret;
	#if 0
	ret = rtk_ponmac_txDisGpioId_get(&gpio_disTx); 
	if(ret != RT_ERR_OK)
    {
        gpio_disTx = CONFIG_TX_DISABLE_GPIO_PIN;
    }
	#else
	gpio_disTx = CONFIG_TX_DISABLE_GPIO_PIN;
	#endif
	
	gpio_disTx_pin = gpio_disTx;
	
	/* set ds_tx gpio to 0 */
	rtk_gpio_databit_set(gpio_disTx, 0);
	/* set ds_tx gpio mode */
	rtk_gpio_mode_set(gpio_disTx, GPIO_OUTPUT);
	/* enable ds_tx gpio */
	rtk_gpio_state_set(gpio_disTx, ENABLED);

	printf("%s gpio_disTx_pin[%d]\n", __func__, gpio_disTx_pin);
}

void ctc_oam_auth_state_init(unsigned char llidIdx)
{
	ctc_oam_onuAuthState_set(llidIdx, CTC_OAM_ONUAUTH_STATE_NOTCOMPLETE, 0);
	ctc_oam_onuAuthType_set(llidIdx, CTC_OAM_ONUAUTH_AUTH_MAC);
}

int ctc_oam_init(void)
{
    int i, ret;
    char nvVarValue[CTC_EXECMD_UV_VALUE_LEN];
    char nvVar[CTC_EXECMD_UV_VAR_LEN];
    unsigned char oui[3] = CTC_OAM_OUI;
    unsigned int tmp0, tmp1, tmp2;
    char ouiStr[10];
    oam_infoOrgSpecCb_t infoCb;
    oam_orgSpecCb_t orgSpecCb;

    //get oui from mib table
    memset(ouiStr, 0, sizeof(ouiStr));
    ret = ctc_oam_flash_var_get(CTC_OAM_MIB_OUI, ouiStr, sizeof(ouiStr));
	if(EPON_OAM_ERR_OK == ret)
	{
	    sscanf(ouiStr, "%2x,%2x,%2x", &tmp0, &tmp1, &tmp2);
	    oui[0] = tmp0;
	    oui[1] = tmp1;
	    oui[2] = tmp2;
	}
    
    /* Register info organization specific callback */
    infoCb.parser = ctc_oam_extInfo_parser;
    infoCb.handler = ctc_oam_extInfo_handler;
    ret = epon_oam_orgSpecCb_reg(
        EPON_OAM_CBTYPE_INFO_ORGSPEC,
        oui,
        (void *) &infoCb);
    if(EPON_OAM_ERR_OK != ret)
    {
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
            "Failed to register CTC info callback functions\n");
        return EPON_OAM_ERR_UNKNOWN;
    }

    /* Register organization specific callback */
    orgSpecCb.processor = ctc_oam_orgSpec_processor;
    ret = epon_oam_orgSpecCb_reg(
        EPON_OAM_CBTYPE_ORGSPEC,
        oui,
        (void *) &orgSpecCb);
    if(EPON_OAM_ERR_OK != ret)
    {
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
            "Failed to register CTC orgSpec callback functions\n");
        return EPON_OAM_ERR_UNKNOWN;
    }

    ret = sem_init(&ctcDataSem, 0, 1);
    if(0 != ret)
    {
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
            "Failed init data semaphore %d %d\n", ret, errno);
    }

    /* Retrive the current image info */
    ret = ctc_oam_swDownloadNv_get("sw_active", nvVarValue);
    /* Lock the data during the init */
    sem_wait(&ctcDataSem);
    if(0 == ret)
    {
        bootInfo.active = atoi(nvVarValue);
        /* Read version according to the active number */
        sprintf(nvVar, "sw_version%d", bootInfo.active);
        ret = ctc_oam_swDownloadNv_get(nvVar, nvVarValue);
        if(0 == ret)
        {
            strncpy(bootInfo.version, nvVarValue, CTC_EXECMD_UV_VALUE_LEN);
        }
        else
        {
            strcpy(bootInfo.version, EPON_OAM_VERSION);
        }
    }
    else
    {
        bootInfo.active = -1;
        strcpy(bootInfo.version, EPON_OAM_VERSION);
    }

    ret = ctc_oam_swDownloadNv_get("sw_commit", nvVarValue);
    if(0 == ret)
    {
        bootInfo.commit = atoi(nvVarValue);
    }
    else
    {
        bootInfo.commit = -1;
    }
    sem_post(&ctcDataSem);

    /* Initial wrappers */
    ctc_wrapper_init();

	ctc_oam_sn_init();

    ctc_oam_alarm_init();

	ctc_oam_silent_mode_init();

	ctc_oam_gpio_tx_disable_init();

#ifdef YUEME_CUSTOMIZED_CHANGE
	ctc_oam_defaultAuthType_init();
#endif

    pthread_mutex_init(&pmThreadMutex, NULL);
    pthread_cond_init(&pmThreadCond, NULL);
    ret = pthread_create(&pmThread, NULL, &ctc_oam_pmThread, NULL);
    if(ret != 0)
    {
        pmThread = 0;
        EPON_OAM_PRINT(EPON_OAM_DBGFLAG_CTC_ERROR,
            "[OAM:%s:%d] performance monitor create fail\n", __FILE__, __LINE__);
    }
    return EPON_OAM_ERR_OK;
}

int ctc_oam_db_init(
    unsigned char llidIdx)
{
    ctc_onuAuthLoid_t loidAuth;

    /* MODIFY ME - all those variables need semaphore protection */
    /* Init extended discovery state */
    ctc_discovery_state[llidIdx] = CTC_OAM_FSM_STATE_WAIT_REMOTE;
    memset(&currCtcVer[llidIdx], 0x0, sizeof(ctc_infoOamVer_t));

	if(isCtcInit == 0)
	{
		/* init ctc_swdl_file content to 0 */
		memset(&ctc_swdl_file, 0x0, sizeof(ctc_swDlFile_t));
		/* only set default loid and password when init first time */
	    memset(&loidAuth, 0x0, sizeof(ctc_onuAuthLoid_t));
	    //strncpy(loidAuth.loid, "user", sizeof("user"));
	    //strncpy(loidAuth.password, "password", sizeof("password"));
        ctc_oam_flash_var_get(CTC_OAM_LOID, loidAuth.loid, sizeof(loidAuth.loid));
        ctc_oam_flash_var_get(CTC_OAM_LOID_PASSWD, loidAuth.password, sizeof(loidAuth.password));
#ifdef YUEME_CUSTOMIZED_CHANGE
		if ('\0' == loidAuth.loid[0])
			epon_oam_isLoidSet_set(llidIdx, 0);
		else
			epon_oam_isLoidSet_set(llidIdx, 1);
#endif
		if (('\0' == loidAuth.loid[0]) && ('\0' == loidAuth.password[0]))
        {
    	    strncpy(loidAuth.loid, "user", sizeof("user"));
    	    strncpy(loidAuth.password, "password", sizeof("password"));
        }
	    ctc_oam_onuAuthLoid_set(llidIdx, &loidAuth);
		ctc_oam_auth_state_init(llidIdx);
		isCtcInit = 1;
	}
	
    /* Only base LLID can use software download */
    if(llidIdx == 0)
    {
        ctc_oam_swDownloadBuf_clear();
    }

    memset(&ctc_last_churningKey[llidIdx], 0x0, sizeof(ctc_churning_t));
	{   /* initialize churning key */
		int i;
		unsigned char churningKey[CTC_ORGSPEC_CHURNING_KEY_10G_LEN];
		ctc_oam_churningKey_rand(churningKey, CTC_ORGSPEC_CHURNING_KEY_10G_LEN);
		for(i = 0; i < CTC_ORGSPEC_CHURNING_KEY_10G_LEN; i++)
    		ctc_last_churningKey[llidIdx].churningKey[i] = churningKey[i];
    }

    /* Disable FEC whenever restarted */
#ifndef CONFIG_CA_RTK_EPON_FEATURE
    CTC_WRAPPER(fecStateSet, DISABLED);
#endif

    return EPON_OAM_ERR_OK;
}

