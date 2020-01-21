// Copyright (c) Acconeer AB, 2015-2019
// All rights reserved

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include "acc_driver_hal.h"
#include "acc_rss.h"
#include "acc_service.h"
#include "acc_service_envelope.h"
#include "acc_sweep_configuration.h"

#include "acc_version.h"

#define REQ_DATA_NUM 128                    // 取得したいデータ数
#define USE_SENSOR_NUM 2                    // 使用するセンサの数
int use_sensor_id[USE_SENSOR_NUM] = {1, 3}; // 使用するセンサのIDリスト
const double period_sec = 0.005;            // 実行周期[s]

double distance_to_object[3] = {0}; // 物体までの距離[m]
int inited = 0;
uint16_t count = 0;

static acc_service_status_t execute_envelope_with_blocking_calls(int sensor_num, acc_service_handle_t handle);
static acc_service_handle_t createHandle(acc_service_configuration_t envelope_configuration);
void writeHeader(acc_service_handle_t handle);
static void configure_sweeps(acc_service_configuration_t envelope_configuration, int id);
static acc_hal_t hal;

double processed_data[1000] = {0};

FILE *fp[3];
FILE *fp_g;

int main(void)
{
    // Nucleoとの通信確立
    int fd = serialOpen("/dev/ttyACM0", 9600);
    wiringPiSetup();
    fflush(stdout);
    if (fd < 0)
    {
        printf("can not open serialport");
    }
    else
    {
        printf("opened Serial Port");
    }

    // ログファイルの生成処理
    char dir_name[64];
    char filename[USE_SENSOR_NUM][64];
    char time_str[32] = "00_00_00_00_00";
    time_t t = time(NULL);
    strftime(time_str, sizeof(time_str), "%m_%d_%H_%M_%S", localtime(&t));
    sprintf(dir_name, "log/%s", time_str);
    // 保存フォルダの生成
    if (mkdir(dir_name, 0777) != 0)
    {
        printf("フォルダ作成に失敗しました。\n");
        return EXIT_FAILURE;
    }
    for (int i = 0; i < USE_SENSOR_NUM; i++)
    {
        // ファイルの生成
        sprintf(filename[i], "%s/%d.csv", dir_name, use_sensor_id[i]);
        if ((fp[i] = fopen(filename[i], "w")) == NULL)
        {
            fprintf(stderr, "ファイルのオープンに失敗しました.\n");
            return EXIT_FAILURE;
        }
    }

    // ファイルの生成
    char gyro_file_name[128];
    sprintf(gyro_file_name, "%s/gyro.csv", dir_name);
    if ((fp_g = fopen(gyro_file_name, "w")) == NULL)
    {
        fprintf(stderr, "ファイルのオープンに失敗しました.\n");
        return EXIT_FAILURE;
    }

    // RADARのドライバを初期化
    if (!acc_driver_hal_init())
        return EXIT_FAILURE;

    hal = acc_driver_hal_get_implementation();
    if (!acc_rss_activate_with_hal(&hal))
        return EXIT_FAILURE;

    // 各種設定
    acc_service_configuration_t envelope_configuration[USE_SENSOR_NUM];
    acc_service_handle_t handle[USE_SENSOR_NUM];
    for (int i = 0; i < USE_SENSOR_NUM; i++)
    {
        envelope_configuration[i] = acc_service_envelope_configuration_create();
        if (envelope_configuration[i] == NULL)
            return EXIT_FAILURE;

        // 移動平均フィルタをなくす
        acc_service_envelope_running_average_factor_set(envelope_configuration[i], 0.0);

        configure_sweeps(envelope_configuration[i], use_sensor_id[i]);

        handle[i] = createHandle(envelope_configuration[i]);
        if (handle[i] == NULL)
            return EXIT_FAILURE;
    }

    clock_t system_start_time = clock();
    long loop_num = 0;

    // ブロッキングコールでエンベロープを実行する
    acc_service_status_t service_status[USE_SENSOR_NUM];
    while (true)
    {
        loop_num++;
        count++;

        // cls
        printf("\033[2J");
        printf("\033[0;0H");

        double now_time = (double)(clock() - system_start_time) / CLOCKS_PER_SEC;
        printf("time: %f\r\n", now_time);

        for (int i = 0; i < USE_SENSOR_NUM; i++)
        {
            service_status[i] = execute_envelope_with_blocking_calls(i, handle[i]);
        }

        if (distance_to_object[0] != 0)
        {
            if (distance_to_object[0] < 0.15)
            {
                serialPutchar(fd, 1);
            }
            else
            {
                serialPutchar(fd, 0);
            }
        }
        else
        {
            serialPutchar(fd, 0);
        }

        int get_char = serialGetchar(fd);
        if (get_char != -1)
        {
            printf("recive : %d\n", get_char);
            fprintf(fp_g, "\n%f", now_time);
            fprintf(fp_g, ",%d", get_char);
        }
        else
        {
            printf("no data\n");
        }

        while (true)
        {
            if ((double)((clock() - system_start_time) / CLOCKS_PER_SEC) > loop_num * period_sec)
                break;
        }
    }

    if (service_status[0] != ACC_SERVICE_STATUS_OK)
    {
        acc_service_envelope_configuration_destroy(&envelope_configuration[0]);
        return EXIT_FAILURE;
    }

    acc_service_envelope_configuration_destroy(&envelope_configuration[0]);

    acc_rss_deactivate();

    return EXIT_SUCCESS;
}

// ハンドルの生成
acc_service_handle_t createHandle(acc_service_configuration_t envelope_configuration)
{
    // ハンドルの生成
    acc_service_handle_t handle = acc_service_create(envelope_configuration);
    if (handle == NULL)
    {
        printf("acc_service_createに失敗\n");
    }

    return handle;
}

double measure_start_dis;
double measure_len;
double measure_end_dis;
uint16_t data_len;
double index_to_meter;

// ブロッキングコールでエンベロープを実行する
acc_service_status_t execute_envelope_with_blocking_calls(int sensor_num, acc_service_handle_t handle)
{
    // メタデータの取得
    acc_service_envelope_metadata_t envelope_metadata;
    acc_service_envelope_get_metadata(handle, &envelope_metadata);
    double measure_start_dis = (double)envelope_metadata.actual_start_m;
    double measure_len = (double)envelope_metadata.actual_length_m;
    double measure_end_dis = (double)(measure_start_dis + measure_len);
    uint16_t data_len = envelope_metadata.data_length;
    double index_to_meter = (double)(measure_len / REQ_DATA_NUM); // [m / point]

    if (inited != USE_SENSOR_NUM)
    {
        printf("始点    : %f [m]\n", measure_start_dis);
        printf("測定距離: %f [m]\n", measure_len);
        printf("終点    : %f [m]\n", measure_end_dis);
        printf("データ長: %u    \n", data_len);
        printf("分解能  : %f [m / point]\n", index_to_meter);

        // 1行目の記入
        for (uint_fast16_t index = 0; index < REQ_DATA_NUM; index++)
        {
            double now_depth = measure_start_dis + index * index_to_meter; // [m]
            fprintf(fp[sensor_num], ",%f", now_depth);
        }

        inited++;
    }

    // センサの状態を取得
    acc_service_status_t service_status = acc_service_activate(handle);

    double sec = (double)count * period_sec;
    fprintf(fp[sensor_num], "\n%f", sec);

    acc_service_envelope_result_info_t result_info;
    // センサの状態に応じて処理を振り分け
    if (service_status == ACC_SERVICE_STATUS_OK)
    {
        uint16_t data[2000];

        // センサから包絡線データを取得する
        // この関数は，次のスイープがセンサーから到着し，包絡線データが「data」配列にコピーされるまでブロックする
        service_status = acc_service_envelope_get_next(handle, data, data_len, &result_info);

        // 取得したデータを間引く
        for (int i = 0; i < REQ_DATA_NUM; i++)
        {
            processed_data[i] = data[(int)(data_len * i / REQ_DATA_NUM)];
        }

        // 減衰補正
        // for (int i = 0; i < data_len; i++)
        // {
        //     data[i] = data[i] + 0.5 * data[i] * (i * index_to_meter);
        // }

        double max_data = 0;
        int max_data_index = 0;
        for (int i = 0; i < data_len; i++)
        {
            if (max_data < data[i])
            {
                max_data_index = i;
                max_data = data[i];
            }
        }

        if (max_data > 1000)
        {
            distance_to_object[sensor_num] = index_to_meter * max_data_index;
        }
        else
        {
            // 強度が閾値より低いときは非検知扱いにする
            distance_to_object[sensor_num] = 0;
        }

        if (sensor_num == 0)
        {
            printf("[%f, %f]\n", max_data, distance_to_object[sensor_num]);
        }

        if (service_status == ACC_SERVICE_STATUS_OK)
        {
            for (uint_fast16_t index = 0; index < REQ_DATA_NUM; index++)
            {
                // printf("%6u", (unsigned int)(data[index]));
                fprintf(fp[sensor_num], ",%lf", processed_data[index]);
            }

            printf("\n");
        }
        else
        {
            printf("エンベロープデータが正しく取得されませんでした\n");
        }
    }
    else
    {
        printf("acc_service_activate() %u => %s\n", (unsigned int)service_status, acc_service_status_name_get(service_status));
    }

    // acc_service_destroy(&handle);

    return service_status;
}

// スイープの設定
void configure_sweeps(acc_service_configuration_t envelope_configuration, int id)
{
    acc_sweep_configuration_t sweep_configuration = acc_service_get_sweep_configuration(envelope_configuration);

    if (sweep_configuration == NULL)
    {
        printf("スイープ構成は利用できません\n");
    }
    else
    {
        float start_m = 0.1f;
        float length_m = 0.7f;
        float update_rate_hz = 100;
        acc_sweep_configuration_sensor_set(sweep_configuration, id);
        acc_sweep_configuration_requested_start_set(sweep_configuration, start_m);
        acc_sweep_configuration_requested_length_set(sweep_configuration, length_m);
        acc_sweep_configuration_repetition_mode_streaming_set(sweep_configuration, update_rate_hz);
    }
}
