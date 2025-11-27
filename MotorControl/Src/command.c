/*============================================================================
    File Name     : command.h
    Description   : 命令解析模块 - 包含命令解析相关的函数
    Author        : ZHOUHENG
    Date          : 2025-11-03
    ----------------------------------------------------------------------       

*=============================================================================
*/
#include "command.h"

#include "motor_params.h"
#include "normalization.h"
//#include "usart.h"

#include <stdint.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char last_error[128];

extern void Vofa_Plot_Start(void);
extern void Vofa_Plot_Stop(void);

#define COMMAND_BUFFER_SIZE 128

typedef enum {
    MOTOR_ID_PARSE_OK = 0,
    MOTOR_ID_PARSE_INVALID,
    MOTOR_ID_PARSE_OUT_OF_RANGE
} motor_id_parse_result_t;

static void clear_last_error(void);
static void set_error(const char* fmt, ...);
static int string_equals_ignore_case(const char* lhs, const char* rhs);
static int starts_with_ignore_case(const char* str, const char* prefix);
static char* skip_leading_whitespace(char* str);
static void rtrim_in_place(char* str);
static void print_available_motors(void);
static motor_id_parse_result_t parse_motor_id_token(const char* token, uint8_t* motor_id_out, long* raw_value_out);
static void handle_plot_command(char* args);
static void handle_motor_command(char* args);
static void handle_set_command(char* args);
static void handle_limit_command(char* args);

void Command_Init(void)
{
    clear_last_error();
}

void Command_Parse(const char* command_line)
{
    if (command_line == NULL) {
        set_error("Command is NULL");
        return;
    }

    clear_last_error();

    size_t length = strlen(command_line);
    if (length == 0U) {
        return;
    }

    if (length >= COMMAND_BUFFER_SIZE) {
        set_error("Command too long");
        return;
    }

    char buffer[COMMAND_BUFFER_SIZE];
    strncpy(buffer, command_line, COMMAND_BUFFER_SIZE - 1U);
    buffer[COMMAND_BUFFER_SIZE - 1U] = '\0';

    char* cursor = skip_leading_whitespace(buffer);
    rtrim_in_place(cursor);

    if (*cursor == '\0') {
        return;
    }

    char* args = cursor;
    while (*args != '\0' && !isspace((unsigned char)*args)) {
        args++;
    }

    if (*args != '\0') {
        *args = '\0';
        args++;
        args = skip_leading_whitespace(args);
    } else {
        args = NULL;
    }

    if (string_equals_ignore_case(cursor, "plot")) {
        handle_plot_command(args);
    } else if (string_equals_ignore_case(cursor, "motor") || string_equals_ignore_case(cursor, "m")) {
        handle_motor_command(args);
    } else if (string_equals_ignore_case(cursor, "set")) {
        handle_set_command(args);
    } else if (string_equals_ignore_case(cursor, "limit")) {
        handle_limit_command(args);
    } else if (starts_with_ignore_case(cursor, "motor")) {
        char* combined_args = args;
        if (combined_args == NULL) {
            combined_args = cursor + 5;
            combined_args = skip_leading_whitespace(combined_args);
            if (combined_args != NULL && *combined_args == '\0') {
                combined_args = NULL;
            }
        }
        handle_motor_command(combined_args);
    } else if ((tolower((unsigned char)cursor[0]) == 'm') && isdigit((unsigned char)cursor[1])) {
        char* combined_args = cursor + 1;
        combined_args = skip_leading_whitespace(combined_args);
        if (combined_args != NULL && *combined_args == '\0') {
            combined_args = NULL;
        }
        handle_motor_command(combined_args);
    } else {
        set_error("Unknown command '%s'", cursor);
    }
}

const char* Command_GetLastError(void)
{
    return last_error;
}

static void clear_last_error(void)
{
    last_error[0] = '\0';
}

static void set_error(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(last_error, sizeof(last_error), fmt, args);
    va_end(args);

    printf("Command Error: %s\r\n", last_error);
}

static int string_equals_ignore_case(const char* lhs, const char* rhs)
{
    if (lhs == NULL || rhs == NULL) {
        return 0;
    }

    while (*lhs != '\0' && *rhs != '\0') {
        if (tolower((unsigned char)*lhs) != tolower((unsigned char)*rhs)) {
            return 0;
        }
        lhs++;
        rhs++;
    }

    return (*lhs == '\0') && (*rhs == '\0');
}

static int starts_with_ignore_case(const char* str, const char* prefix)
{
    if (str == NULL || prefix == NULL) {
        return 0;
    }

    while (*prefix != '\0') {
        if (*str == '\0' || tolower((unsigned char)*str) != tolower((unsigned char)*prefix)) {
            return 0;
        }
        str++;
        prefix++;
    }

    return 1;
}

static char* skip_leading_whitespace(char* str)
{
    if (str == NULL) {
        return NULL;
    }

    while (*str != '\0' && isspace((unsigned char)*str)) {
        str++;
    }

    return str;
}

static void rtrim_in_place(char* str)
{
    if (str == NULL) {
        return;
    }

    size_t len = strlen(str);
    while (len > 0U && isspace((unsigned char)str[len - 1U])) {
        str[len - 1U] = '\0';
        len--;
    }
}

static void print_available_motors(void)
{
    printf("Available motors:");
    for (uint8_t i = 0U; i < motors_number; i++) {
        printf(" %u", i);
    }
    printf("\r\n");
}

static motor_id_parse_result_t parse_motor_id_token(const char* token, uint8_t* motor_id_out, long* raw_value_out)
{
    if (token == NULL || motor_id_out == NULL) {
        return MOTOR_ID_PARSE_INVALID;
    }

    const char* cursor = token;
    while (*cursor != '\0' && isspace((unsigned char)*cursor)) {
        cursor++;
    }

    if (*cursor == '\0') {
        return MOTOR_ID_PARSE_INVALID;
    }

    if (starts_with_ignore_case(cursor, "motor")) {
        cursor += 5;
    } else if ((tolower((unsigned char)*cursor) == 'm') && isdigit((unsigned char)cursor[1])) {
        cursor += 1;
    }

    while (*cursor != '\0' && isspace((unsigned char)*cursor)) {
        cursor++;
    }

    if (*cursor == '\0') {
        return MOTOR_ID_PARSE_INVALID;
    }

    char* endptr = NULL;
    long value = strtol(cursor, &endptr, 10);
    if (cursor == endptr) {
        return MOTOR_ID_PARSE_INVALID;
    }

    while (*endptr != '\0' && isspace((unsigned char)*endptr)) {
        endptr++;
    }

    if (*endptr != '\0') {
        return MOTOR_ID_PARSE_INVALID;
    }

    if (raw_value_out != NULL) {
        *raw_value_out = value;
    }

    if (value < 0 || value >= motors_number) {
        return MOTOR_ID_PARSE_OUT_OF_RANGE;
    }

    *motor_id_out = (uint8_t)value;
    return MOTOR_ID_PARSE_OK;
}

static void handle_plot_command(char* args)
{
    char* trimmed = skip_leading_whitespace(args);
    if (trimmed == NULL || *trimmed == '\0') {
        Vofa_Plot_Start();
        return;
    }

    rtrim_in_place(trimmed);

    if (string_equals_ignore_case(trimmed, "stop")) {
        Vofa_Plot_Stop();
    } else {
        set_error("Invalid plot argument '%s'", trimmed);
    }
}

static void handle_motor_command(char* args)
{
    char* trimmed = skip_leading_whitespace(args);
    if (trimmed == NULL || *trimmed == '\0') {
        set_error("Usage: motor <motor_id>");
        print_available_motors();
        return;
    }

    rtrim_in_place(trimmed);

    uint8_t motor_id = 0U;
    long raw_value = -1;
    motor_id_parse_result_t result = parse_motor_id_token(trimmed, &motor_id, &raw_value);
    if (result == MOTOR_ID_PARSE_OK) {
        MotorParams_PrintAll(motor_id);
        return;
    }

    if (result == MOTOR_ID_PARSE_OUT_OF_RANGE) {
        set_error("Motor %ld not found", raw_value);
    } else {
        set_error("Invalid motor ID '%s'", trimmed);
    }
    print_available_motors();
}

static void handle_enable_command(uint8_t motor_id, bool enable)
{
    if (enable) {
        MotorParams_SetActiveMotor(motor_id);
        // 更新归一化基值
        Normalization_UpdateMotor(motor_id);
    } else {
        MotorParams_DisableMotor(motor_id);
    }
}

// 辅助函数：检查字符串是否以指定子串结尾（大小写不敏感）
static bool ends_with_ignore_case(const char* str, const char* suffix)
{
    if (str == NULL || suffix == NULL) {
        return false;
    }
    
    size_t str_len = strlen(str);
    size_t suffix_len = strlen(suffix);
    
    if (suffix_len > str_len) {
        return false;
    }
    
    return string_equals_ignore_case(str + str_len - suffix_len, suffix);
}

static void handle_set_command(char* args)
{
    char* trimmed = skip_leading_whitespace(args);
    if (trimmed == NULL || *trimmed == '\0') {
        set_error("Usage: set <motor_id> <param> = <value>");
        return;
    }

    rtrim_in_place(trimmed);

    // 检查是否为enable/disable命令（大小写不敏感）
    bool is_enable = false;
    bool is_disable = false;
    
    // 使用大小写不敏感比较检查命令结尾
    if (ends_with_ignore_case(trimmed, "enable")) {
        is_enable = true;
    } else if (ends_with_ignore_case(trimmed, "disable")) {
        is_disable = true;
    }
    
    if (is_enable || is_disable) {
        // 处理enable/disable命令
        size_t trimmed_len = strlen(trimmed);
        size_t keyword_len = is_enable ? 6 : 7; // "enable" 或 "disable" 的长度
        
        // 提取电机ID部分（去掉enable/disable关键字）
        char motor_id_part[COMMAND_BUFFER_SIZE];
        if (trimmed_len > keyword_len) {
            strncpy(motor_id_part, trimmed, trimmed_len - keyword_len);
            motor_id_part[trimmed_len - keyword_len] = '\0';
            
            // 去除尾部空格
            rtrim_in_place(motor_id_part);
            
            // 解析电机ID
            uint8_t motor_index = 0U;
            long raw_id = -1;
            motor_id_parse_result_t parse_result = parse_motor_id_token(motor_id_part, &motor_index, &raw_id);
            
            if (parse_result == MOTOR_ID_PARSE_OK) {
                handle_enable_command(motor_index, is_enable);
                return;
            } else if (parse_result == MOTOR_ID_PARSE_OUT_OF_RANGE) {
                set_error("Motor %ld not found", raw_id);
            } else {
                set_error("Invalid motor ID in enable command");
            }
            print_available_motors();
            return;
        } else {
            set_error("Missing motor ID in enable command");
            print_available_motors();
            return;
        }
    }

    char* equal_sign = strchr(trimmed, '=');
    if (equal_sign == NULL) {
        set_error("Invalid set command: missing '='");
        return;
    }

    char* value_str = equal_sign + 1;
    *equal_sign = '\0';

    value_str = skip_leading_whitespace(value_str);
    rtrim_in_place(value_str);
    if (*value_str == '\0') {
        set_error("Missing value for set command");
        return;
    }

    char* target = skip_leading_whitespace(trimmed);
    rtrim_in_place(target);
    if (*target == '\0') {
        set_error("Missing parameter name");
        return;
    }

    uint8_t motor_index = 0U;
    char* param_name = target;

    char* separator = target;
    while (*separator != '\0' && !isspace((unsigned char)*separator)) {
        separator++;
    }

    if (*separator != '\0') {
        char original = *separator;
        *separator = '\0';
        char* maybe_param = skip_leading_whitespace(separator + 1);
        if (*maybe_param == '\0') {
            set_error("Missing parameter name");
            return;
        }

        long raw_id = -1;
        motor_id_parse_result_t parse_result = parse_motor_id_token(target, &motor_index, &raw_id);
        if (parse_result == MOTOR_ID_PARSE_OK) {
            param_name = maybe_param;
        } else if (parse_result == MOTOR_ID_PARSE_OUT_OF_RANGE ||
                   starts_with_ignore_case(target, "motor") ||
                   starts_with_ignore_case(target, "m") ||
                   isdigit((unsigned char)target[0])) {
            if (parse_result == MOTOR_ID_PARSE_OUT_OF_RANGE) {
                set_error("Motor %ld not found", raw_id);
            } else {
                set_error("Invalid motor ID '%s'", target);
            }
            print_available_motors();
            return;
        } else {
            *separator = original;
            param_name = target;
        }
    } else {
        long raw_id = -1;
        motor_id_parse_result_t parse_result = parse_motor_id_token(target, &motor_index, &raw_id);
        if (parse_result == MOTOR_ID_PARSE_OK) {
            set_error("Missing parameter name");
            return;
        } else if (parse_result == MOTOR_ID_PARSE_OUT_OF_RANGE) {
            set_error("Motor %ld not found", raw_id);
            print_available_motors();
            return;
        }
    }

    param_name = skip_leading_whitespace(param_name);
    rtrim_in_place(param_name);
    if (*param_name == '\0') {
        set_error("Missing parameter name");
        return;
    }

    // 检查是否为限值参数（支持 HMI 编号和参数名）
    // 限值参数包括：I_limit_user, speed_limit_user, position_limit_enable, position_limit_min, position_limit_max
    // 对应 HMI 编号：P2001, P2002, P2003, P2004, P2005
    if (string_equals_ignore_case(param_name, "I_limit_user") ||
        string_equals_ignore_case(param_name, "speed_limit_user") ||
        string_equals_ignore_case(param_name, "position_limit_enable") ||
        string_equals_ignore_case(param_name, "position_limit_min") ||
        string_equals_ignore_case(param_name, "position_limit_max") ||
        string_equals_ignore_case(param_name, "I_limit_max") ||
        (param_name[0] == 'P' && param_name[1] == '2')) {  // P2xxx 格式的 HMI 编号

        // 特殊处理 position_limit_enable，支持 true/false 字符串
        if (string_equals_ignore_case(param_name, "position_limit_enable") ||
            string_equals_ignore_case(param_name, "P2003")) {
            float numeric_value = 0.0f;
            if (string_equals_ignore_case(value_str, "true") ||
                string_equals_ignore_case(value_str, "1") ||
                string_equals_ignore_case(value_str, "enable")) {
                numeric_value = 1.0f;
            } else if (string_equals_ignore_case(value_str, "false") ||
                       string_equals_ignore_case(value_str, "0") ||
                       string_equals_ignore_case(value_str, "disable")) {
                numeric_value = 0.0f;
            } else {
                set_error("Invalid value '%s' for position_limit_enable (use true/false or 1/0)", value_str);
                return;
            }
            MotorParams_SetLimitParam(motor_index, param_name, numeric_value);
            return;
        }

        // 处理其他限值参数（数值类型）
        char* endptr = NULL;
        float value = strtof(value_str, &endptr);
        if (value_str == endptr) {
            set_error("Invalid value '%s'", value_str);
            return;
        }

        while (*endptr != '\0' && isspace((unsigned char)*endptr)) {
            endptr++;
        }

        if (*endptr != '\0') {
            set_error("Invalid value '%s'", value_str);
            return;
        }

        MotorParams_SetLimitParam(motor_index, param_name, value);
        return;
    }

    // 处理电机参数
    char* endptr = NULL;
    float value = strtof(value_str, &endptr);
    if (value_str == endptr) {
        set_error("Invalid value '%s'", value_str);
        return;
    }

    while (*endptr != '\0' && isspace((unsigned char)*endptr)) {
        endptr++;
    }

    if (*endptr != '\0') {
        set_error("Invalid value '%s'", value_str);
        return;
    }

    MotorParams_SetParam(motor_index, param_name, value);
    Normalization_UpdateMotor(motor_index);
}

static void handle_limit_command(char* args)
{
    char* trimmed = skip_leading_whitespace(args);
    if (trimmed == NULL || *trimmed == '\0') {
        // 如果没有指定电机ID，打印激活电机的限值
        if (MotorParams_IsAnyMotorActive()) {
            uint8_t active_motor = MotorParams_GetActiveMotor();
            MotorParams_PrintLimits(active_motor);
        } else {
            set_error("Usage: limit <motor_id> (no active motor)");
            print_available_motors();
        }
        return;
    }

    rtrim_in_place(trimmed);

    uint8_t motor_id = 0U;
    long raw_value = -1;
    motor_id_parse_result_t result = parse_motor_id_token(trimmed, &motor_id, &raw_value);
    if (result == MOTOR_ID_PARSE_OK) {
        MotorParams_PrintLimits(motor_id);
        return;
    }

    if (result == MOTOR_ID_PARSE_OUT_OF_RANGE) {
        set_error("Motor %ld not found", raw_value);
    } else {
        set_error("Invalid motor ID '%s'", trimmed);
    }
    print_available_motors();
}
