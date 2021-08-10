
#include "defines.h"
#include "global.h"
#include "core/ntshell.h"
#include "core/ntlibc.h"
#include "util/ntopt.h"

// シェルコマンド関数郡
static int usrcmd_help(int argc, char **argv);
static int usrcmd_info(int argc, char **argv);
static int usrcmd_imu(int argc, char **argv);
static int usrcmd_sensor(int argc, char **argv);

// NT-Shell用ローカル関数群
static int func_read(char *buf, int cnt, void *extobj);
static int func_write(const char *buf, int cnt, void *extobj);
static int user_callback(const char *text, void *extobj);
static int usrcmd_execute(const char *text);
static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj);

// シェルコマンド用構造体
typedef struct {
    char *cmd;
    char *desc;
    int (*func)(int, char**);
} cmd_table_t;

// コマンドリスト
static const cmd_table_t cmdlist[] = {
    { "help", 			"help command.", 		usrcmd_help 	},
    { "info", 			"system info.", 		usrcmd_info 	},
	{ "imu", 			"IMU debug.",			usrcmd_imu		},
	{ "ir_sensor", 		"IR sensor debug.",		usrcmd_sensor	},
	{ "module_test", 	"module test command.", module_test 	},
};
static ntshell_t nts;

/* ---------------------------------------------------------------
	各ラップ関数
--------------------------------------------------------------- */
static int usrcmd_imu(int argc, char **argv)
{
	IMU_ResetReference();
	IMU_DebugPrintf();
	return 0;
}

static int usrcmd_sensor(int argc, char **argv)
{
	if(argc != 2) {
		printf("  ir_sensor raw\r\n");
		printf("  ir_sensor wall\r\n");
		return 0;
	} else if(ntlibc_strcmp(argv[1], "raw") == 0) {
		Sensor_DebugPrintf();
		return 0;
	} else if (ntlibc_strcmp(argv[1], "wall") == 0) {
		Wall_DebugPrintf();
		return 0;
	} else;
	printf("  Unknown sub command found\r\n");
	return -1;
}

/* ---------------------------------------------------------------
	helpコマンドとinfoコマンドの実行関数
--------------------------------------------------------------- */
static int usrcmd_help(int argc, char **argv)
{
	const cmd_table_t *p = &cmdlist[0];
	for(int i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
		printf("  %s", p->cmd);
		uint8_t space_num = 16 - strlen(p->cmd);
		if( space_num > 0 ) {
			for(int i = 0; i < space_num; i++) {
				printf(" ");
			}
		} else;
		printf(": %s\r\n", p->desc);
		p++;
	}
	return 0;
}

static int usrcmd_info(int argc, char **argv)
{
	if(argc != 2) {
		printf("  info sys\r\n");
		printf("  info ver\r\n");
		return 0;
	} else if(ntlibc_strcmp(argv[1], "sys") == 0) {
		printf("  sylphy_force\r\n");
		return 0;
	} else if (ntlibc_strcmp(argv[1], "ver") == 0) {
		printf("  Version 0.0.1\r\n");
		return 0;
	} else;
	printf("  Unknown sub command found\r\n");
	return -1;
}

/* ---------------------------------------------------------------
	送受信用ローカル関数
--------------------------------------------------------------- */
static int func_read(char *buf, int cnt, void *extobj)
{
	for(int16_t i = 0; i < cnt; i++) {
		buf[i] = (char)Communicate_ReceiceDMA();
	}
	return cnt;
}

static int func_write(const char *buf, int cnt, void *extobj)
{
	for (int i = 0; i < cnt; i++) {
		Communicate_TransmitDMA((uint8_t)(buf[i]));
	}
	return cnt;
}

/* ---------------------------------------------------------------
	コールバック関数
--------------------------------------------------------------- */
static int user_callback(const char *text, void *extobj)
{
	usrcmd_execute(text);
	return 0;
}

static int usrcmd_execute(const char *text)
{
    return ntopt_parse(text, usrcmd_ntopt_callback, 0);
}

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj)
{
    if (argc == 0) {
        return 0;
    }
    const cmd_table_t *p = &cmdlist[0];
    for (int i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
        if (ntlibc_strcmp((const char *)argv[0], p->cmd) == 0) {
            return p->func(argc, argv);
        }
        p++;
    }
    printf("Unknown command found.\r\n");
    return 0;
}

/* ---------------------------------------------------------------
	初期設定関数
--------------------------------------------------------------- */
void Myshell_Initialize( void )
{
	void *extobj = 0;

	ntshell_init(&nts, func_read, func_write, user_callback, extobj);
}

void Myshell_Execute( void )
{
	if( (&nts)->initcode != 0x4367 ) {
		return;
	} else;

	unsigned char ch;
	func_read((char *)&ch, sizeof(ch), (&nts)->extobj);
	vtrecv_execute(&((&nts)->vtrecv), &ch, sizeof(ch));
}
