/**************************************************************************//**
 * @file     main.cpp
 * @version  V1.00
 * @brief    DS-CNN network sample. Demonstrate keyword spotting
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "BoardInit.hpp"      /* Board initialisation */
/* On zephyr, redirect ml-embedded-evaluation-kit logging to zephyr way */
#if defined(__ZEPHYR__)
#define REGISTER_LOG_MODULE_APP 1
#endif
#include "log_macros.h"      /* Logging macros (optional) */

#include "AudioUtils.hpp"
#include "BufAttributes.hpp" /* Buffer attributes to be applied */
#include "Classifier.hpp"    /* Classifier for the result */
#include "ClassificationResult.hpp"
#include "InputFiles.hpp"             /* Baked-in input (not needed for live data) */
#include "KwsModel.hpp"       /* Model API */
#include "Labels.hpp"
#include "MicroNetKwsMfcc.hpp"
#include "KwsProcessing.hpp" /* Pre and Post Process */
#include "KwsResult.hpp"

#undef PI /* PI macro conflict with CMSIS/DSP */
#include "NuMicro.h"

/* On zephyr, configure via Kconfig */
#if defined(__ZEPHYR__)
#include "DMICRecord.h"
#include "Profiler.hpp"
#else
//#define __PROFILE__
#define USE_DMIC
#include "DMICRecord.h"

#if defined(__PROFILE__)
    #include "Profiler.hpp"
#endif
#endif

#if defined(__ZEPHYR__)
#include <zephyr/shell/shell.h>
#if defined(CONFIG_NVT_ML_KWS_OUTPUT_MQTT)
#include "ezmqtt/config.h"
#include "ezmqtt/ezmqtt.h"
#endif
#endif

namespace arm
{
namespace app
{
/* Tensor arena buffer */
static uint8_t tensorArena[ACTIVATION_BUF_SZ] ACTIVATION_BUF_ATTRIBUTE;

/* Optional getter function for the model pointer and its size. */
namespace kws
{
/* On zephyr, use shell instead of unsupported getchar */
#if defined(__ZEPHYR__)

K_MUTEX_DEFINE(mutex_record_ctrl);
K_CONDVAR_DEFINE(condvar_record_ctrl);

static bool record_ctrl_cont;
static bool record_ctrl_oneshot;
static bool record_ctrl_end;

static int kws_next_cmd_handler(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(sh);
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    k_mutex_lock(&mutex_record_ctrl, K_FOREVER);
#if !defined(USE_DMIC)
    record_ctrl_cont = false;
    record_ctrl_oneshot = true;
#else
    record_ctrl_cont = true;
    record_ctrl_oneshot = false;
    shell_print(sh, "Not support one-shot for DMIC\n");
#endif
    k_condvar_signal(&condvar_record_ctrl);
    k_mutex_unlock(&mutex_record_ctrl);

    return 0;
}

static int kws_exit_cmd_handler(const struct shell *sh, size_t argc,
			        char **argv)
{
    ARG_UNUSED(sh);
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    k_mutex_lock(&mutex_record_ctrl, K_FOREVER);
    record_ctrl_cont = false;
    record_ctrl_oneshot = false;
    record_ctrl_end = true;
    k_condvar_signal(&condvar_record_ctrl);
    k_mutex_unlock(&mutex_record_ctrl);

    return 0;
}

static int kws_resume_cmd_handler(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(sh);
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    k_mutex_lock(&mutex_record_ctrl, K_FOREVER);
    record_ctrl_cont = true;
    record_ctrl_oneshot = false;
    k_condvar_signal(&condvar_record_ctrl);
    k_mutex_unlock(&mutex_record_ctrl);

    return 0;
}

static int kws_suspend_cmd_handler(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(sh);
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    k_mutex_lock(&mutex_record_ctrl, K_FOREVER);
    record_ctrl_cont = false;
    record_ctrl_oneshot = false;
    k_condvar_signal(&condvar_record_ctrl);
    k_mutex_unlock(&mutex_record_ctrl);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(kws_subcmd_set,
	SHELL_CMD_ARG(exit, NULL, "Exit kws app", kws_exit_cmd_handler, 1, 0),
	SHELL_CMD_ARG(next, NULL, "Resume kws recording one-shot", kws_next_cmd_handler, 1, 0),
	SHELL_CMD_ARG(resume, NULL, "Resume kws recording continuously", kws_resume_cmd_handler, 1, 0),
	SHELL_CMD_ARG(suspend, NULL, "Suspend kws recording", kws_suspend_cmd_handler, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(kws, &kws_subcmd_set, "kws", NULL);

#endif

extern uint8_t *GetModelPointer();
extern size_t GetModelLen();
} /* namespace kws */
} /* namespace app */
} /* namespace arm */

int main()
{
    /* Initialise the UART module to allow printf related functions (if using retarget) */
    BoardInit();

#if defined(__ZEPHYR__)
    int rc = 0;

#if defined(CONFIG_NVT_ML_KWS_OUTPUT_MQTT)
    static char mqtt_client_id[30];
    static char mqtt_payload[128];
    static char mqtt_topic[60];

    rc = ezmqtt_setup();
    if (rc < 0) {
        printf_err("Failed to initialise mqtt: %d\n", rc);
        return 1;
    }

    rc = ezmqtt_get_client_id(mqtt_client_id, sizeof(mqtt_client_id));
    if (rc < 0) {
        printf_err("Failed to get mqtt client id: %d\n", rc);
        return 1;
    }

    rc = snprintf(mqtt_topic, sizeof(mqtt_topic), "%s/%s", mqtt_client_id, CONFIG_NVT_ML_KWS_MQTT_TOPIC);
    if (rc < 0 || rc >= sizeof(mqtt_topic)) {
        printf_err("Failed to form mqtt topic name\n");
        return 1;
    }
#endif
#endif

    /* Model object creation and initialisation. */
    arm::app::KwsModel model;

    if (!model.Init(arm::app::tensorArena,
                    sizeof(arm::app::tensorArena),
                    arm::app::kws::GetModelPointer(),
                    arm::app::kws::GetModelLen()))
    {
        printf_err("Failed to initialise model\n");
        return 1;
    }

    /*
     * On zephyr, configure mpu region for tensor arena in zephyr way
     * rather than direct control
     */
#if !defined(__ZEPHYR__)
    /* Setup cache poicy of tensor arean buffer */
    info("Set tesnor arena cache policy to WTRA \n");
    const std::vector<ARM_MPU_Region_t> mpuConfig =
    {
        {
            // SRAM for tensor arena
            ARM_MPU_RBAR(((unsigned int)arm::app::tensorArena),        // Base
                         ARM_MPU_SH_NON,    // Non-shareable
                         0,                 // Read-only
                         1,                 // Non-Privileged
                         1),                // eXecute Never enabled
            ARM_MPU_RLAR((((unsigned int)arm::app::tensorArena) + ACTIVATION_BUF_SZ - 1),        // Limit
                         eMPU_ATTR_CACHEABLE_WTRA) // Attribute index - Write-Through, Read-allocate
        }
    };

    // Setup MPU configuration
    InitPreDefMPURegion(&mpuConfig[0], mpuConfig.size());
#endif

    constexpr int minTensorDims = static_cast<int>(
                                      (arm::app::KwsModel::ms_inputRowsIdx > arm::app::KwsModel::ms_inputColsIdx)
                                      ? arm::app::KwsModel::ms_inputRowsIdx
                                      : arm::app::KwsModel::ms_inputColsIdx);

    const auto mfccFrameLength = 640;
    const auto mfccFrameStride = 320;
    const auto scoreThreshold  = 0.75;

    /* Get Input and Output tensors for pre/post processing. */
    TfLiteTensor *inputTensor  = model.GetInputTensor(0);
    TfLiteTensor *outputTensor = model.GetOutputTensor(0);

    if (!inputTensor->dims)
    {
        printf_err("Invalid input tensor dims\n");
        return 2;
    }
    else if (inputTensor->dims->size < minTensorDims)
    {
        printf_err("Input tensor dimension should be >= %d\n", minTensorDims);
        return 3;
    }

    /* Get input shape for feature extraction. */
    TfLiteIntArray *inputShape     = model.GetInputShape(0);
#if defined (MODEL_DS_CNN)
    const uint32_t numMfccFeatures = 10;
    const uint32_t numMfccFrames = 49;
#else
    const uint32_t numMfccFeatures = inputShape->data[arm::app::KwsModel::ms_inputColsIdx];
    const uint32_t numMfccFrames = inputShape->data[arm::app::KwsModel::ms_inputRowsIdx];
#endif

    /* We expect to be sampling 1 second worth of data at a time.
     * NOTE: This is only used for time stamp calculation. */
    const float secondsPerSample = 1.0 / arm::app::audio::MicroNetKwsMFCC::ms_defaultSamplingFreq;

    /* Classifier object for results */
    arm::app::KwsClassifier classifier;

    /* Object to hold label strings. */
    std::vector<std::string> labels;

    /* Populate the labels here. */
    GetLabelsVector(labels);

    uint8_t u8ClipIdx = 0;

#if defined(USE_DMIC)
#define AUDIO_SAMPLE_BLOCK  4
#define AUDIO_SAMPLE_RATE   16000
#define AUDIO_CHANNEL       1


    const auto audioSlidingSamples = (numMfccFrames + 1) * mfccFrameStride; //(49+1)*320 = 16000 = 1 sec
    const auto audioStrideSamples = audioSlidingSamples / 2;

    int16_t *audioSlidingBuf = new int16_t[audioSlidingSamples];
    int32_t ret;

    /* On zephyr, check heap OOM error */
#if defined(__ZEPHYR__)
    if (!audioSlidingBuf)
    {
        printf_err("Allocate DMIC record sliding buffer failed\n");
        return 4;
    }
#endif

    ret = DMICRecord_Init(AUDIO_SAMPLE_RATE, AUDIO_CHANNEL, audioSlidingSamples, AUDIO_SAMPLE_BLOCK);

    if (ret)
    {
        printf_err("Unable init DMIC record error(%d)\n", ret);
        delete []audioSlidingBuf;
        return 4;
    }

#endif


#if defined(__PROFILE__)
    arm::app::Profiler profiler;
    uint64_t u64StartCycle;
    uint64_t u64EndCycle;
#endif


#if defined(__ZEPHYR__)
    warn("Press 'kws next' to resume audio clip inference one-shot\n");
    warn("Press 'kws resume' to resume audio clip inference continuously\n");
    warn("Press 'kws suspend' to suspend audio clip inference\n");
    warn("Press 'kws exit' to exit program\n");

#if defined(CONFIG_NVT_ML_KWS_OUTPUT_MQTT)
    warn("Subscribe to MQTT topic for inference result:\n");
    warn("MQTT server: %s\n", EZMQTT_SERVER_HOSTNAME);
    warn("MQTT topic: %s\n", mqtt_topic);
#endif

#if defined(USE_DMIC)
    DMICRecord_StartRec();
#endif

    while (1)
    {
        k_mutex_lock(&arm::app::kws::mutex_record_ctrl, K_FOREVER);
        if (arm::app::kws::record_ctrl_end) {
            k_mutex_unlock(&arm::app::kws::mutex_record_ctrl);
#if defined(CONFIG_NVT_ML_KWS_OUTPUT_MQTT)
            ezmqtt_teardown();
#endif
            warn("Bye!\n");
            break;
        } else if (!arm::app::kws::record_ctrl_cont && !arm::app::kws::record_ctrl_oneshot) {
            k_condvar_wait(&arm::app::kws::condvar_record_ctrl, &arm::app::kws::mutex_record_ctrl, K_FOREVER);
            k_mutex_unlock(&arm::app::kws::mutex_record_ctrl);
            continue;
        } else if (arm::app::kws::record_ctrl_cont) {
            __ASSERT_NO_MSG(!arm::app::kws::record_ctrl_oneshot);
        } else if (arm::app::kws::record_ctrl_oneshot) {
            __ASSERT_NO_MSG(!arm::app::kws::record_ctrl_cont);
            arm::app::kws::record_ctrl_oneshot = false;
        }
        k_mutex_unlock(&arm::app::kws::mutex_record_ctrl);
#else
#if !defined(USE_DMIC)
    char chStdIn;

    info("Press 'n' to run next audio clip inference \n");
    info("Press 'q' to exit program \n");

    while ((chStdIn = getchar()))
    {
        if (chStdIn == 'q')
            break;
        else if (chStdIn != 'n')
            continue;

#else
    DMICRecord_StartRec();

    while (1)
    {
#endif
#endif

        /* Declare a container to hold results from across the whole audio clip. */
        std::vector<arm::app::kws::KwsResult> finalResults;

        /* Object to hold classification results */
        std::vector<arm::app::ClassificationResult> singleInfResult;

        /* Set up pre and post-processing. */
        arm::app::KwsPreProcess preProcess = arm::app::KwsPreProcess(
                                                 inputTensor, numMfccFeatures, numMfccFrames, mfccFrameLength, mfccFrameStride);

    /* On zephyr, KwsPostProcess has change on passing 'useSoftmax' */
#if defined(__ZEPHYR__)
#if defined(MODEL_DS_CNN)
        arm::app::KwsPostProcess postProcess =
            arm::app::KwsPostProcess(outputTensor, classifier, labels, singleInfResult, 0, false);
#else
        arm::app::KwsPostProcess postProcess =
            arm::app::KwsPostProcess(outputTensor, classifier, labels, singleInfResult, 0, true);
#endif
#else
#if defined(MODEL_DS_CNN)
        arm::app::KwsPostProcess postProcess =
            arm::app::KwsPostProcess(outputTensor, classifier, labels, singleInfResult, false);
#else
        arm::app::KwsPostProcess postProcess =
            arm::app::KwsPostProcess(outputTensor, classifier, labels, singleInfResult, true);
#endif
#endif

#if defined(USE_DMIC)

        while (DMICRecord_AvailSamples() < audioSlidingSamples)
        {
            /*
             * On zephyr, other threads will starve with busy-wait here,
	     * for example, logging thread won't work with lower priority.
	     * Change to non-busy-wait.
	     */
#if defined(__ZEPHYR__)
            DMICRecord_SleepUntilUpdate(200);
#else
            __NOP();
#endif
        }

        DMICRecord_ReadSamples(audioSlidingBuf, audioSlidingSamples);

        DMICRecord_UpdateReadSampleIndex(audioStrideSamples);

        /* Creating a sliding window through the whole audio clip. */
        auto audioDataSlider = arm::app::audio::SlidingWindow<const int16_t>(
                                   audioSlidingBuf,
                                   audioSlidingSamples,
                                   preProcess.m_audioDataWindowSize, preProcess.m_audioDataStride);

#else
        /* Creating a sliding window through the whole audio clip. */
        auto audioDataSlider = arm::app::audio::SlidingWindow<const int16_t>(
                                   get_audio_array(u8ClipIdx),
                                   get_audio_array_size(u8ClipIdx),
                                   preProcess.m_audioDataWindowSize, preProcess.m_audioDataStride);

        info("Using audio data from %s\n", get_filename(u8ClipIdx));
#endif

        /* Start sliding through audio clip. */
        while (audioDataSlider.HasNext())
        {
            const int16_t *inferenceWindow = audioDataSlider.Next();

            /* KwsPreProcess has updated on passing inference index */
#if !defined(__ZEPHYR__)
            /* The first window does not have cache ready. */
            preProcess.m_audioWindowIndex = audioDataSlider.Index();
#endif

            info("Inference %zu/%zu\n", audioDataSlider.Index() + 1,
                 audioDataSlider.TotalStrides() + 1);

#if defined(__PROFILE__)
            u64StartCycle = pmu_get_systick_Count();
#endif

            /* Run the pre-processing, inference and post-processing. */
#if defined(__ZEPHYR__)
            if (!preProcess.DoPreProcess(inferenceWindow, audioDataSlider.Index()))
#else
            if (!preProcess.DoPreProcess(inferenceWindow, arm::app::audio::MicroNetKwsMFCC::ms_defaultSamplingFreq))
#endif
            {
                printf_err("Pre-processing failed.");
                break;
            }

#if defined(__PROFILE__)
            u64EndCycle = pmu_get_systick_Count();
            info("MFCC cycles %llu \n", (u64EndCycle - u64StartCycle));
#endif


#if defined(__PROFILE__)
            profiler.StartProfiling("Inference");
#endif

            if (!model.RunInference())
            {
                printf_err("Inference failed.");
                break;
            }

#if defined(__PROFILE__)
            profiler.StopProfiling();
#endif

#if defined(__PROFILE__)
            u64StartCycle = pmu_get_systick_Count();
#endif

            if (!postProcess.DoPostProcess())
            {
                printf_err("Post-processing failed.");
                break;
            }

#if defined(__PROFILE__)
            u64EndCycle = pmu_get_systick_Count();
            info("Post process cycles %llu \n", (u64EndCycle - u64StartCycle));
#endif

            /* Add results from this window to our final results vector. */
            finalResults.emplace_back(arm::app::kws::KwsResult(singleInfResult,
                                                               audioDataSlider.Index() * secondsPerSample * preProcess.m_audioDataStride,
                                                               audioDataSlider.Index(), scoreThreshold));

        } /* while (audioDataSlider.HasNext()) */

        for (const auto &result : finalResults)
        {

            std::string topKeyword{"<none>"};
            float score = 0.f;

            if (!result.m_resultVec.empty())
            {
                topKeyword = result.m_resultVec[0].m_label;
                score      = result.m_resultVec[0].m_normalisedVal;
            }

            if (result.m_resultVec.empty())
            {
#if 0
                info("For timestamp: %f (inference #: %" PRIu32 "); label: %s; threshold: %f\n",
                     result.m_timeStamp,
                     result.m_inferenceNumber,
                     topKeyword.c_str(),
                     result.m_threshold);
#endif
            }
            else
            {
                for (uint32_t j = 0; j < result.m_resultVec.size(); ++j)
                {
#if defined(__ZEPHYR__)
#if defined(CONFIG_NVT_ML_KWS_OUTPUT_MQTT)
		    snprintf(mqtt_payload, sizeof(mqtt_payload),
		             "For timestamp: %f (inference #: %" PRIu32
                             "); label: %s, score: %f; threshold: %f\n",
                             result.m_timeStamp,
                             result.m_inferenceNumber,
                             result.m_resultVec[j].m_label.c_str(),
                             result.m_resultVec[j].m_normalisedVal,
                             result.m_threshold);
                    mqtt_payload[sizeof(mqtt_payload) - 1] = '\0';
		    rc = ezmqtt_publish_str(mqtt_topic, mqtt_payload);
		    if (rc == 0) {
                        /* MQTT publish OK */
		    } else if (rc == -EAGAIN) {
                        warn("Publish mqtt timed out\n");
                    } else {
                        printf_err("Failed to publish mqtt: %d\n", rc);
                        arm::app::kws::record_ctrl_end = true;
		    }
#endif
#endif
                    info("For timestamp: %f (inference #: %" PRIu32
                         "); label: %s, score: %f; threshold: %f\n",
                         result.m_timeStamp,
                         result.m_inferenceNumber,
                         result.m_resultVec[j].m_label.c_str(),
                         result.m_resultVec[j].m_normalisedVal,
                         result.m_threshold);
                }
            }
        }

#if defined(__PROFILE__)
        profiler.PrintProfilingResult();
#endif

        u8ClipIdx ++;

        if (u8ClipIdx >= NUMBER_OF_FILES)
            u8ClipIdx = 0;

    }

#if defined(USE_DMIC)
    delete []audioSlidingBuf;
#else
    return 0;
#endif
}
