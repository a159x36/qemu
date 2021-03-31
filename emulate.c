#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void copyfile(char *name, FILE *fout, int loc) {
    FILE* fin=fopen(name,"rb");
    if(fin==0) {
       perror(0);
       exit(1);
    }
    fseek(fout,loc,SEEK_SET);
    while(!feof(fin)) {
        int buf[256];
        int n=fread(buf,4,256,fin);
        fwrite(buf, 4, n, fout);
    }
    fclose(fin);
}

int main(int argc, char*argv[]) {
    int ard=0;
    if(argc<3) {
        printf("emulate firmware build_dir\n");
        exit(1);
    }
    if(argc==5 && !strcmp(argv[4],"arduino")) {
	ard=1;
    }
    char *firmware_name=argv[1];
    char *build_dir=argv[2];
    char *package_dir=argv[3];

    char bootloader_name[256];
    char partitions_name[256];
    char boot_app_name[256];
    char cmd[256];
    if(!ard)
      snprintf(bootloader_name,256,"%s/bootloader.bin",build_dir);
    else {
      snprintf(bootloader_name,256,"%s/framework-arduinoespressif32/tools/sdk/bin/bootloader_dio_40m.bin",package_dir);
      snprintf(boot_app_name,256,"%s/framework-arduinoespressif32/tools/partitions/boot_app0.bin",package_dir);
    }
    snprintf(partitions_name,256,"%s/partitions.bin",build_dir);
//    printf("%s\n",argv[0]);
    char package_path[128];
    strcpy(package_path,argv[0]);
    int l=strlen(package_path);
    while(l>0 && package_path[l]!='/' && package_path[l]!='\\') l--;
    package_path[l]=0;

    snprintf(cmd,256,"%s/xtensa-softmmu/qemu-system-xtensa -machine esp32 -drive file=esp32flash.bin,if=mtd,format=raw -display default,show-cursor=on"
            ,package_path);

    FILE* fout=fopen("esp32flash.bin","wb");
    copyfile(bootloader_name, fout, 0x1000);
    copyfile(partitions_name, fout, 0x8000);
    if(ard) copyfile(boot_app_name, fout, 0xe000);
    copyfile(firmware_name, fout, 0x10000);
    fseek(fout,0x3fffff,SEEK_SET);
    int x=0;
    fwrite(&x, 1, 1, fout);
    fclose(fout);
    system(cmd);   

    return 0;
}
