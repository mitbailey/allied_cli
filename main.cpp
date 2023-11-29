/**
 * @file main.cpp
 * @author Mit Bailey (mitbailey@outlook.com)
 * @brief
 * @version See Git tags for version information.
 * @date 2023.11.28
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <czmq.h>
#include <alliedcam.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>
#include <aDIO_library.h>
#include <map>
#include <exception>
#include <stdarg.h>
#include <signal.h>

#include "meb_print.h"
#include "stringhasher.hpp"
#include "string_format.hpp"

volatile sig_atomic_t done = 0;

void sighandler(int sig)
{
    done = 1;
}

class CameraInfo
{
public:
    std::string idstr;
    std::string name;
    std::string model;
    std::string serial;

    CameraInfo()
    {
        idstr = "";
        name = "";
        model = "";
        serial = "";
    }

    CameraInfo(VmbCameraInfo_t info)
    {
        idstr = info.cameraIdString;
        name = info.cameraName;
        model = info.modelName;
        serial = info.serialString;
    }

    CameraInfo(VmbCameraInfo_t *info)
    {
        idstr = info->cameraIdString;
        name = info->cameraName;
        model = info->modelName;
        serial = info->serialString;
    }
};

enum CommandNames
{
    image_format = 100,       // string
    sensor_bit_depth = 101,   // string
    trigline = 102,           // string
    trigline_src = 103,       // string
    exposure_us = 104,        // double
    acq_framerate = 105,      // double
    acq_framerate_auto = 106, // bool
    image_size = 200,         // special, two arguments, ints
    image_ofst = 201,         // special, two arguments, ints
    sensor_size = 202,
    throughput_limit = 300,       // int
    throughput_limit_range = 301, // special
    adio_bit = 10,                // special
};

#define SET_CASE_STR(NAME)                                   \
    case CommandNames::NAME:                                 \
    {                                                        \
        err = allied_set_##NAME(image_cam.handle, argument); \
        break;                                               \
    }

#define SET_CASE_INT(NAME)                              \
    case CommandNames::NAME:                            \
    {                                                   \
        long arg = atol(argument);                      \
        err = allied_set_##NAME(image_cam.handle, arg); \
        break;                                          \
    }

#define SET_CASE_DBL(NAME)                              \
    case CommandNames::NAME:                            \
    {                                                   \
        double arg = atof(argument);                    \
        err = allied_set_##NAME(image_cam.handle, arg); \
        break;                                          \
    }

#define SET_CASE_BOOL(NAME)                             \
    case CommandNames::NAME:                            \
    {                                                   \
        char *narg = strdup(argument);                  \
        for (int i = 0; narg[i]; i++)                   \
        {                                               \
            narg[i] = tolower(narg[i]);                 \
        }                                               \
        bool arg = streq(narg, "true");                 \
        err = allied_set_##NAME(image_cam.handle, arg); \
        break;                                          \
    }

#define GET_CASE_STR(NAME)                                              \
    case CommandNames::NAME:                                            \
    {                                                                   \
        char *garg;                                                     \
        err = allied_get_##NAME(image_cam.handle, (const char **)&garg); \
        reply = garg;                                                   \
        break;                                                          \
    }

#define GET_CASE_DBL(NAME)                                \
    case CommandNames::NAME:                              \
    {                                                     \
        double garg;                                      \
        err = allied_get_##NAME(image_cam.handle, &garg); \
        reply = string_format("%.6f", garg);              \
        break;                                            \
    }

#define GET_CASE_INT(NAME)                                \
    case CommandNames::NAME:                              \
    {                                                     \
        VmbInt64_t garg;                                  \
        err = allied_get_##NAME(image_cam.handle, &garg); \
        reply = std::to_string(garg);                     \
        break;                                            \
    }

#define GET_CASE_BOOL(NAME)                               \
    case CommandNames::NAME:                              \
    {                                                     \
        VmbBool_t garg;                                   \
        err = allied_get_##NAME(image_cam.handle, &garg); \
        reply = garg == VmbBoolTrue ? "True" : "False";   \
        break;                                            \
    }

class CharContainer
{
private:
    char *strdup(const char *str)
    {
        int len = strlen(str);
        char *out = new char[len + 1];
        strcpy(out, str);
        return out;
    }

public:
    char **arr = nullptr;
    int narr = 0;
    int selected;
    size_t maxlen = 0;

    ~CharContainer()
    {
        if (arr)
        {
            for (int i = 0; i < narr; i++)
            {
                delete[] arr[i];
            }
            delete[] arr;
        }
    }

    CharContainer()
    {
        arr = nullptr;
        narr = 0;
        selected = -1;
    }

    CharContainer(const char **arr, int narr)
    {
        this->arr = new char *[narr];
        this->narr = narr;
        this->selected = -1;
        for (int i = 0; i < narr; i++)
        {
            this->arr[i] = strdup(arr[i]);
            if (strlen(arr[i]) > maxlen)
            {
                maxlen = strlen(arr[i]);
            }
        }
    }

    CharContainer(const char **arr, int narr, const char *key)
    {
        this->arr = new char *[narr];
        this->narr = narr;
        for (int i = 0; i < narr; i++)
        {
            this->arr[i] = strdup(arr[i]);
            if (strlen(arr[i]) > maxlen)
            {
                maxlen = strlen(arr[i]);
            }
        }
        this->selected = find_idx(key);
    }

    int find_idx(const char *str)
    {
        int res = -1;
        for (int i = 0; i < narr; i++)
        {
            if (strcmp(arr[i], str) == 0)
                res = i;
        }
        return res;
    }
};

class ImageCam
{
    bool opened = false;
    unsigned char state = 0;
    bool capturing;
    DeviceHandle adio_hdl = nullptr;
    CameraInfo info;

public:
    int adio_bit = -1;
    AlliedCameraHandle_t handle = nullptr;

    ImageCam()
    {
        handle = nullptr;
        capturing = false;
    }

    ImageCam(CameraInfo &camera_info, DeviceHandle adio_hdl)
    {
        handle = nullptr;
        capturing = false;
        this->adio_hdl = adio_hdl;
        this->info = camera_info;
        if (allied_open_camera(&handle, info.idstr.c_str(), 5) != VmbErrorSuccess)
        {
            dbprintlf(FATAL "Failed to open camera %s.", camera_info.idstr.c_str());
            throw std::runtime_error("Failed to open camera.");
        }
    }

    ~ImageCam()
    {
        close_camera();
    }

    static void Callback(const AlliedCameraHandle_t handle, const VmbHandle_t stream, VmbFrame_t *frame, void *user_data)
    {
        assert(user_data);

        ImageCam *self = (ImageCam *)user_data;

        if (self->adio_hdl != nullptr && self->adio_bit >= 0)
        {
            self->state = ~self->state;
            WriteBit_aDIO(self->adio_hdl, 0, self->adio_bit, self->state);
        }

        // self->stat.update();
        // self->img.update(frame);
    }

    void open_camera()
    {
        std::string errmsg = "";
        VmbError_t err = allied_open_camera(&handle, info.idstr.c_str(), 5);
        if (err != VmbErrorSuccess)
        {
            errmsg = "Could not open camera: " + std::string(allied_strerr(err));
            dbprintlf(FATAL "%s", errmsg.c_str());
            return;
        }
        CharContainer *triglines = nullptr;
        CharContainer *trigsrcs = nullptr;
        char *key = nullptr;
        char **arr = nullptr;
        VmbUint32_t narr = 0;
        err = allied_get_trigline(handle, (const char **)&key);
        if (err == VmbErrorSuccess)
        {
            err = allied_get_triglines_list(handle, &arr, NULL, &narr);
            if (err == VmbErrorSuccess)
            {
                triglines = new CharContainer((const char **)arr, narr, (const char *)key);
                free(arr);
                narr = 0;
            }
            else
            {
                dbprintlf("Could not get trigger lines list: %s", allied_strerr(err));
            }
        }
        else
        {
            dbprintlf("Could not get selected trigger line: %s", allied_strerr(err));
        }
        if (triglines != nullptr)
        {
            // set all trigger lines to output
            for (int i = 0; i < triglines->narr; i++)
            {
                char *line = triglines->arr[i];
                err = allied_set_trigline(handle, line);
                if (err != VmbErrorSuccess)
                {
                    dbprintlf("Could not select line %s: %s", line, allied_strerr(err));
                }
                else
                {
                    err = allied_set_trigline_mode(handle, "Output");
                    if (err != VmbErrorSuccess)
                        dbprintlf("Could not set line %s to output: %s", line, allied_strerr(err));
                }
            }
            err = allied_set_trigline(handle, key);
            if (err != VmbErrorSuccess)
                dbprintlf("Could not select line %s: %s", key, allied_strerr(err));
            // get trigger source
            err = allied_get_trigline_src(handle, (const char **)&key);
            if (err == VmbErrorSuccess)
            {
                err = allied_get_trigline_src_list(handle, &arr, NULL, &narr);
                if (err == VmbErrorSuccess)
                {
                    trigsrcs = new CharContainer((const char **)arr, narr, (const char *)key);
                    free(arr);
                    narr = 0;
                }
                else
                {
                    dbprintlf("Could not get trigger sources list: %s", allied_strerr(err));
                }
            }
        }
        delete triglines;
        delete trigsrcs;
        opened = true;
        // std::cout << "Opened!" << std::endl;
    }

    void cleanup()
    {
        if (opened)
        {
            allied_stop_capture(handle);  // just stop capture...
            allied_close_camera(&handle); // close the camera
            opened = false;
        }
    }

    void close_camera()
    {
        cleanup();
        opened = false;
    }

    bool running()
    {
        return capturing;
    }

    VmbError_t start_capture()
    {
        VmbError_t err = VmbErrorSuccess;
        if (handle != nullptr && !capturing)
        {
            err = allied_start_capture(handle, &Callback, (void *)this); // set the callback here
        }
        return err;
    }

    VmbError_t stop_capture()
    {
        VmbError_t err = VmbErrorSuccess;
        if (handle != nullptr && capturing)
        {
            err = allied_stop_capture(handle);
            if (adio_hdl != nullptr && adio_bit >= 0)
            {
                this->state = 0;
                WriteBit_aDIO(adio_hdl, 0, adio_bit, this->state);
            }
        }
        return err;
    }
};

int main(int argc, char *argv[])
{
    // signal handler
    signal(SIGINT, sighandler);
    // arguments
    int adio_minor_num = 0;
    int port = 5555;
    std::string camera_id = "";
    // Argument parsing
    {
        int c;
        while ((c = getopt(argc, argv, "c:a:h")) != -1)
        {
            switch (c)
            {
            case 'c':
            {
                printf("Camera ID from command line: %s\n", optarg);
                camera_id = optarg;
                break;
            }
            case 'a':
            {
                printf("ADIO minor number: %s\n", optarg);
                adio_minor_num = atoi(optarg);
                break;
            }
            case 'p':
            {
                printf("Port number: %s\n", optarg);
                port = atoi(optarg);
                if (port < 5000 || port > 65535)
                {
                    dbprintlf(RED_FG "Invalid port number: %d", port);
                    exit(EXIT_FAILURE);
                }
                break;
            }
            case 'h':
            default:
            {
                printf("\nUsage: %s [-c Camera ID] [-a ADIO Minor Device] [-p ZMQ Port] [-h Show this message]\n\n", argv[0]);
                exit(EXIT_SUCCESS);
            }
            }
        }
    }
    // Create the pipe name
    char *pipe_name = zsys_sprintf("tcp://*:%d", port);
    assert(pipe_name);
    // Set up ADIO
    DeviceHandle adio_dev = nullptr;
    if (OpenDIO_aDIO(&adio_dev, adio_minor_num) != 0)
    {
        dbprintlf(RED_FG "Could not initialize ADIO API. Check if /dev/rtd-aDIO* exists. aDIO features will be disabled.");
        adio_dev = nullptr;
    }
    else
    {
        // set up port A as output and set all bits to low
        int ret = LoadPort0BitDir_aDIO(adio_dev, 1, 1, 1, 1, 1, 1, 1, 1);
        if (ret == -1)
        {
            dbprintlf(RED_FG "Could not set PORT0 to output.");
        }
        else
        {
            ret = WritePort_aDIO(adio_dev, 0, 0); // set all to low
            if (ret < 0)
            {
                dbprintlf(RED_FG "Could not set all PORT0 bits to LOW: %s [%d]", strerror(ret), ret);
            }
        }
    }
    // Initialize String Hasher
    StringHasher hasher = StringHasher();
    // Set up cameras
    std::vector<uint32_t> camids;
    std::map<uint32_t, CameraInfo> caminfos;
    std::map<uint32_t, ImageCam> imagecams;

    VmbError_t err = allied_init_api(NULL);
    if (err != VmbErrorSuccess)
    {
        dbprintlf(FATAL "Failed to initialize Allied Vision API: %s", allied_strerr(err));
        return 1;
    }

    VmbUint32_t count;
    VmbCameraInfo_t *vmbcaminfos;
    err = allied_list_cameras(&vmbcaminfos, &count);
    if (err != VmbErrorSuccess)
    {
        dbprintlf(FATAL "Failed to list cameras: %s", allied_strerr(err));
        return 1;
    }

    for (VmbUint32_t idx = 0; idx < count; idx++)
    {
        CameraInfo caminfo = CameraInfo(vmbcaminfos[idx]);
        uint32_t hash = hasher.get_hash(caminfo.idstr);
        dbprintlf("Camera %d: %s", idx, caminfo.idstr.c_str());
        dbprintlf("Camera %d: %s", idx, caminfo.name.c_str());
        dbprintlf("Camera %d: %s", idx, caminfo.model.c_str());
        dbprintlf("Camera %d: %s", idx, caminfo.serial.c_str());
        imagecams.insert(std::pair<uint32_t, ImageCam>(hash, ImageCam(caminfo, adio_dev)));
        caminfos.insert(std::pair<uint32_t, CameraInfo>(hash, caminfo));
        camids.push_back(hash);
    }

    // Setup ZMQ.
    zsock_t *pipe = zsock_new_rep(pipe_name);
    assert(pipe);
    zstr_free(&pipe_name);
    zpoller_t *poller = zpoller_new(pipe, NULL);
    assert(poller);
    // Loop, waiting for ZMQ commands and performing them as necessary.
    while (!done)
    {
        zsock_t *which = (zsock_t *)zpoller_wait(poller, 1000); // wait a second
        if (which == NULL)
        {
            continue;
        }
        zmsg_t *message = zmsg_recv(which);

        bool get_cmd = false;
        bool set_cmd = false;

        char *cam_id = NULL;
        char *command = NULL;
        char *argument = NULL;
        std::string reply = "None";
        uint32_t chash = 0;

        VmbError_t err = VmbErrorSuccess;

        char *cmd_type = zmsg_popstr(message); // get cmd type
        if (streq(cmd_type, "quit"))
        {
            done = 1;
        }
        else if (streq(cmd_type, "list"))
        {
            // list cameras
            reply = "[";
            for (auto &hash : camids)
            {
                reply += std::to_string(hash) + ", ";
            }
            err = VmbErrorSuccess;
        }
        else if (streq(cmd_type, "start_capture_all"))
        {
            err = VmbErrorSuccess;
            for (auto &image_cam : imagecams)
            {
                err = image_cam.second.start_capture();
                if (err != VmbErrorSuccess)
                {
                    break;
                }
            }
        }
        else if (streq(cmd_type, "stop_capture_all"))
        {
            err = VmbErrorSuccess;
            for (auto &image_cam : imagecams)
            {
                err = image_cam.second.stop_capture();
                if (err != VmbErrorSuccess)
                {
                    break;
                }
            }
        }
        else if (streq(cmd_type, "start_capture"))
        {
            cam_id = zmsg_popstr(message);   // get camera ID
            chash = hasher.get_hash(cam_id); // get camera hash
            try
            {
                ImageCam image_cam = imagecams.at(chash);
                err = image_cam.start_capture(); // do this for specific camera id
            }
            catch (const std::out_of_range &oor)
            {
                err = VmbErrorNotFound;
            }
        }
        else if (streq(cmd_type, "stop_capture"))
        {
            cam_id = zmsg_popstr(message);   // get camera ID
            chash = hasher.get_hash(cam_id); // get camera hash
            try
            {
                ImageCam image_cam = imagecams.at(chash);
                err = image_cam.stop_capture(); // do this for specific camera id
            }
            catch (const std::out_of_range &oor)
            {
                err = VmbErrorNotFound;
            }
        }
        else if (streq(cmd_type, "get"))
        {
            cam_id = zmsg_popstr(message);  // get camera ID
            command = zmsg_popstr(message); // get command
            get_cmd = true;
        }
        else if (streq(cmd_type, "set"))
        {
            cam_id = zmsg_popstr(message);   // get camera ID
            command = zmsg_popstr(message);  // get command
            argument = zmsg_popstr(message); // get argument
            set_cmd = true;
        }
        else
        {
            err = VmbErrorWrongType; // wrong command
        }
        long cmd_num = atol(command);
        if (set_cmd)
        {
            uint32_t chash = hasher.get_hash(cam_id);
            try
            {
                ImageCam image_cam = imagecams.at(chash);
                switch (cmd_num)
                {
                    SET_CASE_STR(image_format)
                    SET_CASE_STR(sensor_bit_depth)
                    SET_CASE_STR(trigline)
                    SET_CASE_STR(trigline_src)
                    SET_CASE_DBL(exposure_us)
                    SET_CASE_DBL(acq_framerate)
                    SET_CASE_BOOL(acq_framerate_auto)
                    SET_CASE_INT(throughput_limit)
                case CommandNames::image_size:
                {
                    char *arg2 = zmsg_popstr(message);
                    long arg1l = atol(argument);
                    long arg2l = atol(arg2);
                    err = allied_set_image_size(image_cam.handle, arg1l, arg2l);
                    zstr_free(&arg2);
                    break;
                }
                case CommandNames::image_ofst:
                {
                    char *arg2 = zmsg_popstr(message);
                    long arg1l = atol(argument);
                    long arg2l = atol(arg2);
                    err = allied_set_image_ofst(image_cam.handle, arg1l, arg2l);
                    zstr_free(&arg2);
                    break;
                }
                case CommandNames::adio_bit:
                {
                    long arg1l = atol(argument);
                    image_cam.adio_bit = arg1l;
                    break;
                }
                default:
                {
                    err = VmbErrorWrongType; // wrong command
                    break;
                }
                }
            }
            catch (const std::out_of_range &oor)
            {
                err = VmbErrorNotFound;
            }
        }
        else if (get_cmd)
        {
            uint32_t chash = hasher.get_hash(cam_id);
            try
            {
                ImageCam image_cam = imagecams.at(chash);
                switch (cmd_num)
                {
                    GET_CASE_STR(image_format)
                    GET_CASE_STR(sensor_bit_depth)
                    GET_CASE_STR(trigline)
                    GET_CASE_STR(trigline_src)
                    GET_CASE_DBL(exposure_us)
                    GET_CASE_DBL(acq_framerate)
                    GET_CASE_BOOL(acq_framerate_auto)
                    GET_CASE_INT(throughput_limit)
                case CommandNames::sensor_size:
                {
                    VmbInt64_t width = 0, height = 0;
                    err = allied_get_sensor_size(image_cam.handle, &width, &height);
                    reply = std::to_string(width) + "x" + std::to_string(height);
                    break;
                }
                case CommandNames::image_size:
                {
                    VmbInt64_t width = 0, height = 0;
                    err = allied_get_image_size(image_cam.handle, &width, &height);
                    reply = std::to_string(width) + "x" + std::to_string(height);
                    break;
                }
                case CommandNames::image_ofst:
                {
                    VmbInt64_t width = 0, height = 0;
                    err = allied_get_image_ofst(image_cam.handle, &width, &height);
                    reply = std::to_string(width) + "x" + std::to_string(height);
                    break;
                }
                case CommandNames::adio_bit:
                {
                    reply = std::to_string(image_cam.adio_bit);
                    break;
                }
                case CommandNames::throughput_limit_range:
                {
                    VmbInt64_t vmin = 0, vmax = 0;
                    err = allied_get_throughput_limit_range(image_cam.handle, &vmin, &vmax, NULL);
                    reply = "[" + std::to_string(vmin) + ", " + std::to_string(vmax) + "]";
                    break;
                }
                default:
                {
                    err = VmbErrorWrongType; // wrong command
                    break;
                }
                }
            }
            catch (const std::out_of_range &oor)
            {
                err = VmbErrorNotFound;
            }
        }
        char *ack_nac = (char *) "ACK";
        if (err != VmbErrorSuccess)
        {
            ack_nac = (char *) "NAC";
        }
        zmsg_t *ack = zmsg_new();
        zmsg_pushstr(ack, ack_nac);       // ack or nack
        zmsg_pushstrf(ack, "%d", err);    // error code
        zmsg_pushstr(ack, reply.c_str()); // None if not set
        zmsg_pushstr(ack, cmd_type);      // command type
        if (cam_id != NULL)
        {
            zmsg_pushstr(ack, cam_id);
            if (command != NULL)
                zmsg_pushstr(ack, command);
        }
        zmsg_send(&ack, which);
        // cleanup
        zstr_free(&cmd_type);
        zstr_free(&cam_id);
        zstr_free(&command);
        zstr_free(&argument);
        zmsg_destroy(&message);
    }

    zpoller_destroy(&poller);
    zsock_destroy(&pipe);

    if (adio_dev != nullptr)
        CloseDIO_aDIO(adio_dev);

    return 0;
}