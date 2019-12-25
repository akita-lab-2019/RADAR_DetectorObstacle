// Copyright (c) Acconeer AB, 2015-2019
// All rights reserved

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include "acc_driver_hal.h"
#include "acc_rss.h"
#include "acc_service.h"
#include "acc_service_envelope.h"
#include "acc_sweep_configuration.h"

#include "acc_version.h"

#define USE_SENSOR_NUM 3                       // 使用するセンサの数
int use_sensor_id[USE_SENSOR_NUM] = {1, 3, 4}; // 使用するセンサのIDリスト
const double period_sec = 0.05;                // 実行周期[s]

double distance_to_object[3] = {0}; // 物体までの距離[m]
int inited = 0;
uint16_t count = 0;

static acc_service_status_t execute_envelope_with_blocking_calls(int sensor_num, acc_service_handle_t handle);
static acc_service_handle_t createHandle(acc_service_configuration_t envelope_configuration);
void writeHeader(acc_service_handle_t handle);
static void configure_sweeps(acc_service_configuration_t envelope_configuration, int id);
static acc_hal_t hal;

FILE *fp[3];

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
    char filename[3][64];
    char time_str[32] = "00_00_00_00_00";
    time_t t = time(NULL);
    strftime(time_str, sizeof(time_str), "%m_%d_%H_%M_%S", localtime(&t));
    for (int i = 0; i < USE_SENSOR_NUM; i++)
    {
        sprintf(filename[i], "log/%s_%d.csv", time_str, use_sensor_id[i]);
        if ((fp[i] = fopen(filename[i], "w")) == NULL)
        {
            fprintf(stderr, "ファイルのオープンに失敗しました.\n");
            return EXIT_FAILURE;
        }
    }

    // RADARのドライバを初期化
    if (!acc_driver_hal_init())
        return EXIT_FAILURE;

    hal = acc_driver_hal_get_implementation();
    if (!acc_rss_activate_with_hal(&hal))
        return EXIT_FAILURE;

    // 各種設定
    acc_service_configuration_t envelope_configuration[3];
    acc_service_handle_t handle[3];
    for (int i = 0; i < USE_SENSOR_NUM; i++)
    {
        envelope_configuration[i] = acc_service_envelope_configuration_create();
        if (envelope_configuration[i] == NULL)
            return EXIT_FAILURE;

        configure_sweeps(envelope_configuration[i], use_sensor_id[i]);

        handle[i] = createHandle(envelope_configuration[i]);
        if (handle[i] == NULL)
            return EXIT_FAILURE;
    }

    // ブロッキングコールでエンベロープを実行する
    acc_service_status_t service_status[3];
    while (true)
    {
        service_status[0] = execute_envelope_with_blocking_calls(0, handle[0]);
        service_status[1] = execute_envelope_with_blocking_calls(1, handle[1]);
        service_status[2] = execute_envelope_with_blocking_calls(2, handle[2]);
        count++;

        if (distance_to_object[0] < 0.1)
        {
            serialPutchar(fd, 1);
        }
        else
        {
            serialPutchar(fd, 0);
        }

        hal.os.sleep_us(period_sec * 1000000);
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
    double index_to_meter = (double)(measure_len / data_len); // [m / point]

    if (inited != USE_SENSOR_NUM)
    {
        printf("始点    : %f [m]\n", measure_start_dis);
        printf("測定距離: %f [m]\n", measure_len);
        printf("終点    : %f [m]\n", measure_end_dis);
        printf("データ長: %u    \n", data_len);
        printf("分解能  : %f [m / point]\n", index_to_meter);

        // 1行目の記入
        for (uint_fast16_t index = 0; index < data_len; index++)
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

        distance_to_object[sensor_num] = index_to_meter * max_data_index;
        printf("%f", distance_to_object[sensor_num]);

        if (service_status == ACC_SERVICE_STATUS_OK)
        {
            for (uint_fast16_t index = 0; index < data_len; index++)
            {
                // printf("%6u", (unsigned int)(data[index]));
                fprintf(fp[sensor_num], ",%u", data[index]);
            }

            printf("\n");
        }
        else
        {
            printf("エンベロープデータが正しく取得されませんでした\n");
        }

        // 測定を終了
        service_status = acc_service_deactivate(handle);
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
        float length_m = 1.5f;
        float update_rate_hz = 100;
        acc_sweep_configuration_sensor_set(sweep_configuration, id);
        acc_sweep_configuration_requested_start_set(sweep_configuration, start_m);
        acc_sweep_configuration_requested_length_set(sweep_configuration, length_m);
        acc_sweep_configuration_repetition_mode_streaming_set(sweep_configuration, update_rate_hz);
    }
}
