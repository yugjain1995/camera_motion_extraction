
/// [headers]
#include<image_subscriber.h>
/// [headers]

int main(int argc, char **argv){
    RosToCvmat img_conv;
    img_conv.imageSubscriber(true, argc, argv);
    return 0;
}