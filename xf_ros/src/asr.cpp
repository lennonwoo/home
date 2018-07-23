#include "xf_ros/asr.hpp"


namespace xf_ros {

struct result {
    std::string slot;
    std::string content;
    int confidence;
};

std::ostream&operator << (std::ostream &o, const struct result r) {
    o << "slot: " << r.slot << std::endl;
    o << "content: " << r.content << std::endl;
    o << "confidence: " << r.confidence << std::endl;
}

std::vector<int> split(std::string str, char delimiter) {
  std::vector<int> internal;
  std::stringstream ss(str); // Turn the string into a stream.
  std::string tok;

  while(getline(ss, tok, delimiter)) {
    internal.push_back(std::stoi(tok));
  }

  return internal;
}

// parse result demo in C++
// but maybe parse it in python is more simple
void parse_result(const char *result) {
    using namespace std;
    string rawtext;
    int totalConfidence;
    vector<struct result> resultList;

    TiXmlDocument doc;
    TiXmlElement *pRoot, *pResult, *pProcess;

    if (result) {
        doc.Parse(result, 0, TIXML_ENCODING_UTF8);
        pRoot = doc.FirstChildElement("nlp");
        if (pRoot) {
            pResult = pRoot->FirstChildElement("result");

            string confidence_ = pResult->FirstChildElement("confidence")->GetText();
            auto confidenceList = split(confidence_, '|');

            int i;
            TiXmlElement *e;
            for (i = 0, e = pResult->FirstChildElement("object")->FirstChildElement();
                    e != NULL;
                    i += 1, e = e->NextSiblingElement()) {
                struct result r = {
                        e->Value(),
                        e->GetText(),
                        confidenceList[i]
                };
                resultList.push_back(r);
            }

            rawtext = pRoot->FirstChildElement("rawtext")->GetText();
            totalConfidence = atoi(pRoot->FirstChildElement("confidence")->GetText());
        }
        cout << rawtext << endl;
        cout << totalConfidence << endl;

        for (auto r : resultList)
            cout << r << endl;
    }
}

void on_result(const char *result, char is_last) {
    printf("[on_result] result returned..\n");

    std_msgs::String msg;

    std::stringstream ss;
    ss << result;
    msg.data = ss.str();
    std::cout << msg.data << std::endl;

    asr_pub.publish(msg);
}

void on_speech_begin() {
    printf("Start Listening...\n");
}

void on_speech_end(int reason) {
    // seem don't have on_sppech_end callback..
    if (reason == END_REASON_VAD_DETECT)
        printf("Speaking end\n\n");
    else
        printf("Recognizer error %d\n\n", reason);
}

void ASR::asr_mic(const char *session_begin_params) {
    int errcode;
    int i = 0;

    struct speech_rec iat;

    struct speech_rec_notifier recnotifier = {
            on_result,
            on_speech_begin,
            on_speech_end
    };

    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
    if (errcode) {
        printf("speech recognizer init failed\n");
        return;
    }
    errcode = sr_start_listening(&iat);
    if (errcode) {
        printf("start listen failed %d\n", errcode);
    }
    /* demo 10 minutes recording */
    while(i++ < 60 * asrContinueMinutes_)
        sleep(1);
    errcode = sr_stop_listening(&iat);
    if (errcode) {
        printf("stop listening failed %d\n", errcode);
    }

    sr_uninit(&iat);
    exit(0);
}

// static void asr_file(const char *audio_file, const char *session_begin_params) {
//     int errcode = 0;
//     FILE*   f_pcm = NULL;
//     char*   p_pcm = NULL;
//     unsigned long   pcm_count = 0;
//     unsigned long   pcm_size = 0;
//     unsigned long   read_size = 0;
//     struct speech_rec iat;
//     struct speech_rec_notifier recnotifier = {
//             on_result,
//             on_speech_begin,
//             on_speech_end
//     };

//     if (NULL == audio_file)
//         goto iat_exit;

//     f_pcm = fopen(audio_file, "rb");
//     if (NULL == f_pcm)
//     {
//         printf("\nopen [%s] failed! \n", audio_file);
//         goto iat_exit;
//     }

//     fseek(f_pcm, 0, SEEK_END);
//     pcm_size = ftell(f_pcm);
//     fseek(f_pcm, 0, SEEK_SET);

//     p_pcm = (char *)malloc(pcm_size);
//     if (NULL == p_pcm)
//     {
//         printf("\nout of memory! \n");
//         goto iat_exit;
//     }

//     read_size = fread((void *)p_pcm, 1, pcm_size, f_pcm);
//     if (read_size != pcm_size)
//     {
//         printf("\nread [%s] error!\n", audio_file);
//         goto iat_exit;
//     }

//     errcode = sr_init(&iat, session_begin_params, SR_USER, &recnotifier);
//     if (errcode) {
//         printf("speech recognizer init failed : %d\n", errcode);
//         goto iat_exit;
//     }

//     errcode = sr_start_listening(&iat);
//     if (errcode) {
//         printf("\nsr_start_listening failed! error code:%d\n", errcode);
//         goto iat_exit;
//     }

//     while (1)
//     {
//         unsigned int len = 10 * FRAME_LEN; /* 200ms audio */
//         int ret = 0;

//         if (pcm_size < 2 * len)
//             len = pcm_size;
//         if (len <= 0)
//             break;

//         ret = sr_write_audio_data(&iat, &p_pcm[pcm_count], len);

//         if (0 != ret)
//         {
//             printf("\nwrite audio data failed! error code:%d\n", ret);
//             goto iat_exit;
//         }

//         pcm_count += (long)len;
//         pcm_size -= (long)len;
//     }

//     errcode = sr_stop_listening(&iat);
//     if (errcode) {
//         printf("\nsr_stop_listening failed! error code:%d \n", errcode);
//         goto iat_exit;
//     }

// iat_exit:
//     if (NULL != f_pcm)
//     {
//         fclose(f_pcm);
//         f_pcm = NULL;
//     }
//     if (NULL != p_pcm)
//     {
//         free(p_pcm);
//         p_pcm = NULL;
//     }

//     //sr_stop_listening(&iat);
//     sr_uninit(&iat);
//     //exit(0);
// }


int build_grm_callback(int ecode, const char *info, void *udata) {
    UserData *grm_data = (UserData *)udata;

    if (NULL != grm_data) {
        grm_data->build_fini = 1;
        grm_data->errcode = ecode;
    }

    if (MSP_SUCCESS == ecode && NULL != info) {
        printf("构建语法成功！ 语法ID:%s\n", info);
        if (NULL != grm_data)
            snprintf(grm_data->grammar_id, MAX_GRAMMARID_LEN - 1, info);
    }
    else
        ROS_ERROR("build grammar failed %d", ecode);

    return 0;
}

int ASR::build_grammar() {
    FILE *grm_file                           = NULL;
    char *grm_content                        = NULL;
    unsigned int grm_cnt_len                 = 0;
    char grm_build_params[MAX_PARAMS_LEN]    = {NULL};
    int ret                                  = 0;

    grm_file = fopen(grmFilePath_.c_str(), "rb");
    if(NULL == grm_file) {
        printf("打开\"%s\"文件失败！[%s]\n", grmFilePath_.c_str(), strerror(errno));
        return -1;
    }

    fseek(grm_file, 0, SEEK_END);
    grm_cnt_len = ftell(grm_file);
    fseek(grm_file, 0, SEEK_SET);

    grm_content = (char *)malloc(grm_cnt_len + 1);
    if (NULL == grm_content)
    {
        printf("内存分配失败!\n");
        fclose(grm_file);
        grm_file = NULL;
        return -1;
    }
    fread((void*)grm_content, 1, grm_cnt_len, grm_file);
    grm_content[grm_cnt_len] = '\0';
    fclose(grm_file);
    grm_file = NULL;

    snprintf(grm_build_params, MAX_PARAMS_LEN - 1,
             "engine_type = local, \
              asr_res_path = %s, sample_rate = %d, \
              grm_build_path = %s, ",
             asrResPath_.c_str(),
             SAMPLE_RATE_16K,
             grmBuildPath_.c_str()
    );
    ret = QISRBuildGrammar("bnf", grm_content, grm_cnt_len, grm_build_params, build_grm_callback, &asr_data_);

    free(grm_content);
    grm_content = NULL;

    return ret;
}


int ASR::run_asr() {
    char asr_params[MAX_PARAMS_LEN]    = {NULL};
    const char *rec_rslt               = NULL;
    const char *session_id             = NULL;
    const char *asr_audiof             = NULL;
    FILE *f_pcm                        = NULL;
    char *pcm_data                     = NULL;
    long pcm_count                     = 0;
    long pcm_size                      = 0;
    int last_audio                     = 0;

    int aud_stat                       = MSP_AUDIO_SAMPLE_CONTINUE;
    int ep_status                      = MSP_EP_LOOKING_FOR_SPEECH;
    int rec_status                     = MSP_REC_STATUS_INCOMPLETE;
    int rss_status                     = MSP_REC_STATUS_INCOMPLETE;
    int errcode                        = -1;
    int aud_src                        = 0;
    //离线语法识别参数设置
    snprintf(asr_params, MAX_PARAMS_LEN - 1,
             "engine_type = local, language = zh_cn, \
              asr_res_path = %s, sample_rate = %d, \
              grm_build_path = %s, local_grammar = %s, \
              result_type = xml, result_encoding = UTF-8, ",
             asrResPath_.c_str(),
             SAMPLE_RATE_16K,
             grmBuildPath_.c_str(),
             asr_data_.grammar_id
    );
    asr_mic(asr_params);
    // asr_file(audioFilePath_.c_str(), asr_params);
    return 0;
}

ASR::ASR(ros::NodeHandle nh)
    : nodeHandle_(nh)
{
    ROS_INFO("[XF ASR] Node started.");

    initParameters();
    initXF();
    run_asr();
}

ASR::~ASR() {
    ROS_INFO("[XF ASR] Node ending...");
    MSPLogout();
}

void ASR::initParameters() {
    nodeHandle_.param("publishers/asr/topic", asrPublisherTopicName_,
                      std::string("/xf_ros/asr_msg"));
    nodeHandle_.param("publishers/asr/queue_size", asrPublisherQueueSize_, 1);
    nodeHandle_.param("publishers/asr/latch", asrPublisherLatch_, false);

    nodeHandle_.param("asr_res_path", asrResPath_,
                      std::string("asr_res_path file unknown"));
    nodeHandle_.param("grm_build_path", grmBuildPath_,
                      std::string("grm_build_path file unknown"));
    nodeHandle_.param("grm_file_path", grmFilePath_,
                      std::string("grm_file_path file unknown"));
    nodeHandle_.param("login_param", loginParams_, std::string("appid = 123456"));
    nodeHandle_.param("audio_file_path", audioFilePath_, std::string("audio file unknown"));
    nodeHandle_.param("asr_continue_minutes", asrContinueMinutes_, 1);

    memset(&asr_data_, 0, sizeof(UserData));
}

void ASR::initXF() {
    //第一个参数为用户名，第二个参数为密码，传NULL即可，第三个参数是登录参数
    checkRet(MSPLogin(NULL, NULL, loginParams_.c_str()), "login failed");

    //第一次使用某语法进行识别，需要先构建语法网络，获取语法ID，之后使用此语法进行识别，无需再次构建
    checkRet(build_grammar(), "build grammar failed");

    while (1 != asr_data_.build_fini)
        usleep(300 * 1000);
    checkRet(asr_data_.errcode);
}

inline void ASR::checkRet(const int ret, const char *errorMsg) {
    if (MSP_SUCCESS != ret) {
        if (errorMsg) {
            ROS_ERROR("[XF ASR] %s error code=%d", errorMsg, ret);
        } else {
            ROS_ERROR("[XF ASR] error code=%d", ret);
        }
        delete this;
    }
}

}
