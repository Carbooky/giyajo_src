
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <termios.h>

#include <curl/curl.h>
#include "util.h"
#include "libctools/xml_utils.h"

#if 0
#define CA_FILE  "CA/Works_Systems_CA.pem"

int Transfer_uploadFile_multi(const char* dir, char** filename, int item_num, const char* url, const char* username,
		const char* password) {

	if (filename == NULL)
		return -1;

	CURL *curl;
	CURLcode res;

	struct stat file_info;
	FILE *fd;

	char user_passwd[64] = "";
	char full_filename[64] = "";

	int i;
	char path_name[256], file_name[256];
	
	// Perform uploading
	dprintf("uploadFile() Target URL: %s,  dir: %s, filename: %s\n", url, dir, filename[0]);

	if (dir != NULL) {
		sprintf(full_filename, "%s/%s", dir, filename[0]);
	} else {
		strcpy(full_filename, filename[0]);
	}

#ifdef HTTP_POST_UPLOAD
	struct curl_httppost *formpost = NULL;
	struct curl_httppost *lastptr = NULL;
#else
	fd = fopen(full_filename, "rb");

	if (!fd) {
		return -1;
	}

	if (fstat(fileno(fd), &file_info) != 0) {
		fclose(fd);
		return -1;
	}
#endif

	fd = fopen(full_filename, "rb");

	if (!fd) {
		return -1;
	}

	if (fstat(fileno(fd), &file_info) != 0) {
		fclose(fd);
		return -1;
	}

	curl = curl_easy_init();

	if (curl) {
		sprintf(user_passwd, "%s:%s", username, password);
		dprintf("file name: %s, password: %s -- upload file begin... \n", full_filename, user_passwd);

		//enable Http Authentication
		{
			curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_DIGEST|CURLAUTH_BASIC);
			curl_easy_setopt(curl, CURLOPT_USERPWD, user_passwd);
		}

		//enable SSL
		{
			curl_easy_setopt(curl, CURLOPT_CAINFO, CA_FILE);
			curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2);
			curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, FALSE);
		}

		curl_easy_setopt(curl, CURLOPT_URL, url);

#ifdef HTTP_POST_UPLOAD
		
		for(i=0; i<item_num; i++){
			// decice file name
			sprintf(file_name, "%s", filename[i]);
			// decide path to file
			sprintf(path_name, "%s/%s", dir, filename[i]);
			/* Fill in the file upload field */
			curl_formadd(&formpost, &lastptr, CURLFORM_COPYNAME, file_name, CURLFORM_FILE, path_name, CURLFORM_END);
			printf("After curl_formadd, file_name=%s, path_name=%s\n", file_name, path_name);
			puts("--------------------------------------------------------");
			puts("--------------------------------------------------------");
			puts("--------------------------------------------------------");
			puts("--------------------------------------------------------");
		}
		curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);

		res = curl_easy_perform(curl);

		/* always cleanup */
		curl_easy_cleanup(curl);

		/* then cleanup the formpost chain */
		curl_formfree(formpost);

#else
		curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

		curl_easy_setopt(curl, CURLOPT_READDATA, fd);

		/* and give the size of the upload (optional) */
		curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, (curl_off_t)file_info.st_size);

		res = curl_easy_perform(curl);

		fclose(fd);

		/* always cleanup */
		curl_easy_cleanup(curl);
#endif

		if (res != CURLE_OK) {
			dprintf("Upload file failed. filenmae = %s, fault code = %d", full_filename, res);
		} else {
			dprintf("Upload file success. filenmae = %s", full_filename);
		}
	}

	return res;
}
#endif

int syscall_getRsp(const char* command, char* returnString) {
  FILE *fp;
  //int status;
  char path[1024];

  /* Open the command for reading. */
  fp = popen(command, "r");
  if (fp == NULL) {
    printf("Failed to run command\n" );
    return -1;
  }

  /* Read the output a line at a time - output it. */
  while (fgets(path, sizeof(path)-1, fp) != NULL) {
    strcpy(returnString, path);
  }

  /* close */
  pclose(fp);
  
  return 1;
}

void ITRI_Hex2Str( uint8_t *pStr, uint8_t decValue  ){
char gaHexValue[]="0123456789abcdef";
uint8_t num;
    for(num=1; num<2; num--){
    	pStr[num] = gaHexValue[decValue & 0x0f];
    	decValue = decValue >> 4;
	}
}

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int append_FCS(uint8_t * FrameBuffer){
	int32_t idx=0;
  uint8_t FCS;
  uint8_t frm_len = 0;
  
  frm_len = FrameBuffer[FRAMEFORMAT_LOC_LEN];
  
  if(frm_len==0)
  	return -1;
  
	FCS=FrameBuffer[FRAMEFORMAT_LOC_LEN];
	for (idx=FRAMEFORMAT_LOC_API_ID;idx<frm_len+2;idx++)
	{
		FCS=FCS^FrameBuffer[idx];
	}
	FrameBuffer[frm_len+2]=FCS;

		return 1;
}

int check_FCS(uint8_t * FrameBuffer){
    int32_t idx=0;
    uint8_t FCS;
		uint8_t frm_len=0;

		frm_len = FrameBuffer[FRAMEFORMAT_LOC_LEN];

    FCS=FrameBuffer[FRAMEFORMAT_LOC_LEN];
    for (idx=FRAMEFORMAT_LOC_API_ID;idx<frm_len+2;idx++)
    {
        FCS=FCS^FrameBuffer[idx];
    }

    if(FrameBuffer[frm_len+2] == FCS)
        return 1;
    else
        return -1;
}

uint8_t get_group(void * frame){
	uint8_t *in_frame = (void*)frame;

	if(in_frame[FRAMEFORMAT_LOC_API_ID]=='\0')
		return 0xff;
	else	
		return in_frame[FRAMEFORMAT_LOC_API_ID];
}

uint8_t get_api_cmd(void * frame){
	uint8_t *in_frame = (void*)frame;

	if(in_frame[FRAMEFORMAT_LOC_OPCODE]=='\0')
		return 0xff;
	else	
		return in_frame[FRAMEFORMAT_LOC_OPCODE];
}

uint8_t * get_api_data(void* frame){
	uint8_t *in_frame = (void*)frame;
	
	//return &in_frame[FRAMEFORMAT_LOC_DATAFIELD_START];	
	return &in_frame[FRAMEFORMAT_LOC_DATA_LEN];
}

uint8_t get_cmd(void* frame){
	uint8_t *in_frame = (void*) frame;
	uint8_t rtn_cmd=0;

	if (frame == NULL)
		return 0xff;

	rtn_cmd = in_frame[FRAMEFORMAT_LOC_DATA_CMD_RTN];
	
		return rtn_cmd;

}

uint8_t get_frameLen(void* frame){
	uint8_t *in_frame = (void*) frame;
	uint8_t rtn_len=0;

	rtn_len = in_frame[FRAMEFORMAT_LOC_LEN];
	
	if (rtn_len ==0)
		return -1;
	else
		return (rtn_len);

}

uint8_t create_rtn_template(raw_pkg_t* in_pkg){

	in_pkg->sync = SYNC_SYMBOL;
	in_pkg->len = 0;

	switch(get_group((void*)in_pkg)){
		case WB_PKG_TYPE_SOLAR_GPIO_READ_RSP:
			in_pkg->len = sizeof(struct _Solar_GPIOReadRsp);	
			in_pkg->uniFrame.Solar_GPIOReadRsp.GroupId=OPGROUP_GPIO;
			in_pkg->uniFrame.Solar_GPIOReadRsp.OpCode=OPCODE_GPIOREAD_RSP;
			in_pkg->uniFrame.Solar_GPIOReadRsp.Length=2;
			//in_pkg->uniFrame.Solar_GPIOReadRsp.ReturnCode=RETURN_CODE_SUCCESS;
  	
			break;
			
		case WB_PKG_TYPE_SOLAR_GPIO_WRITE_CONF:
			in_pkg->len = sizeof(struct _Solar_GPIOWriteConf);
			in_pkg->uniFrame.Solar_GPIOWriteConf.GroupId=OPGROUP_GPIO;
			in_pkg->uniFrame.Solar_GPIOWriteConf.OpCode=OPCODE_GPIOWRITE_CONF;
			in_pkg->uniFrame.Solar_GPIOWriteConf.Length=1;
			//in_pkg->uniFrame.Solar_GPIOWriteConf.ReturnCode=RETURN_CODE_SUCCESS;
  	
			break;

		case WB_PKG_TYPE_SOLAR_VIDEO_CONF:
			in_pkg->len = sizeof(struct _Solar_VideoCtrlReq);
			in_pkg->uniFrame.Solar_VideoCtrlReq.GroupId=OPGROUP_GPIO;
			in_pkg->uniFrame.Solar_VideoCtrlReq.OpCode=OPCODE_VIDEOCTRL_CONF;
			in_pkg->uniFrame.Solar_VideoCtrlReq.Length=1;
			//in_pkg->uniFrame.Solar_GPIOWriteConf.ReturnCode=RETURN_CODE_SUCCESS;
  	
			break;

		case WB_PKG_TYPE_SOLAR_ENERGYREAD_RSP:
		  in_pkg->len = sizeof(struct _Solar_EnergyReadRsp);
			in_pkg->uniFrame.Solar_EnergyReadRsp.OpCode=OPCODE_GPIOREAD_RSP;
			in_pkg->uniFrame.Solar_EnergyReadRsp.Length=0;
			//in_pkg->uniFrame.Solar_EnergyReadRsp.ReturnCode=RETURN_CODE_SUCCESS;
			//in_pkg->uniFrame.Solar_EnergyReadRsp.Value=fill_value;
			
			break;
		default:
			printf("build_gpio_rtn_template failed !!\n");
			return -1;
	}
	return 1;
}


int inv_meta_data_dump( inverter_info_t * data_ptr){
	int i;
	
	double total_p, power_l; 
	
	if(!data_ptr)
		return 0;
	
	// w85
	//switch_bytes(&data_ptr->W_0x85, 1);
	printf("W85 = 0x%x\n",data_ptr->W_0x85);
	
	
	data_ptr->brand_name[15] = '\0';
	data_ptr->type_name[15] = '\0';
	data_ptr->sn_name[15] = '\0';
	printf("Brand Name = %s\n",data_ptr->brand_name);
	printf("Tyte Name = %s\n",data_ptr->type_name);
	printf("SN Name = %s\n",data_ptr->sn_name);
	
	// Total output time
	printf("Total output time = %d:%d:%d\n", data_ptr->inverter_meta_data.TIME_HR_CNT, data_ptr->inverter_meta_data.TIME_MIN_CNT, data_ptr->inverter_meta_data.TIME_SEC_CNT );
	
	// Total Yield
	
	total_p = data_ptr->inverter_meta_data.TOTAL_POWER_H;
	total_p = total_p*1000;
	
	power_l = data_ptr->inverter_meta_data.TOTAL_POWER_L;
	power_l = power_l*0.1;
	
	total_p +=power_l;
	printf("Energy Generated = %.2f kWh\n", total_p );
	
	total_p = total_p * 5;

	// Earning = Total Yield * 5
	printf("Earning = NT$ %.2f \n", total_p);
	
	// Pac
	printf("AC Total Power = %d W\n", data_ptr->inverter_meta_data.Pac );
	
	// Pdc
	printf("DC Total Power = %d W\n", data_ptr->inverter_meta_data.Pdc );
	
	//puts("Raw data of meta_data:\n");
	printf("\n");
	for(i=0; i<98; i++){
		printf("0x%x ",((uint16_t*) &(data_ptr->inverter_meta_data.State))[i]);
	}
	
	return 1;
}

int update_inv_msg2host(inverter_msg2host_t *msg2host, inverter_info_t *inverter_info){
	
	msg2host->state_code = (uint8_t)inverter_info->inverter_meta_data.State;
	memcpy(msg2host->brand_name, inverter_info->brand_name, 16);
	memcpy(msg2host->type_name, inverter_info->type_name, 16);
	memcpy(msg2host->sn_name, inverter_info->sn_name, 16);

	msg2host->total_hr_h = (uint8_t)(inverter_info->inverter_meta_data.TIME_HR_CNT >> 8);
	msg2host->total_hr_l = (uint8_t)(inverter_info->inverter_meta_data.TIME_HR_CNT & 0x00ff);
	msg2host->total_min_h = (uint8_t)(inverter_info->inverter_meta_data.TIME_MIN_CNT >> 8);
	msg2host->total_min_l = (uint8_t)(inverter_info->inverter_meta_data.TIME_MIN_CNT & 0x00ff);
	msg2host->gen_H_h = (uint8_t)(inverter_info->inverter_meta_data.TOTAL_POWER_H >> 8);
	msg2host->gen_H_l = (uint8_t)(inverter_info->inverter_meta_data.TOTAL_POWER_H & 0x00ff);
	msg2host->gen_L_h = (uint8_t)(inverter_info->inverter_meta_data.TOTAL_POWER_L >> 8);
	msg2host->gen_L_l = (uint8_t)(inverter_info->inverter_meta_data.TOTAL_POWER_L & 0x00ff);
	msg2host->pac_h = (uint8_t)(inverter_info->inverter_meta_data.Pac >> 8);
	msg2host->pac_l = (uint8_t)(inverter_info->inverter_meta_data.Pac & 0x00ff);
	msg2host->pdc_h = (uint8_t)(inverter_info->inverter_meta_data.Pdc >> 8);
	msg2host->pdc_l = (uint8_t)(inverter_info->inverter_meta_data.Pdc & 0x00ff);

	return 1;
}

int update_inv_msg2host_new(inverter_msg2host_new_t *msg2host, inverter_info_t *inverter_info, uint8_t invId){
	
	msg2host->state_code = (uint8_t)inverter_info->inverter_meta_data.State;
	msg2host->inv_id = invId;
	memcpy(msg2host->brand_name, inverter_info->brand_name, 16);
	memcpy(msg2host->type_name, inverter_info->type_name, 16);
	memcpy(msg2host->sn_name, inverter_info->sn_name, 16);

	msg2host->total_hr = inverter_info->inverter_meta_data.TIME_HR_CNT;
	msg2host->total_min = inverter_info->inverter_meta_data.TIME_MIN_CNT;
	msg2host->total_sec = inverter_info->inverter_meta_data.TIME_SEC_CNT;
	msg2host->total_gen_h = inverter_info->inverter_meta_data.TOTAL_POWER_H;
	msg2host->total_gen_l = inverter_info->inverter_meta_data.TOTAL_POWER_L;
	msg2host->pac = inverter_info->inverter_meta_data.Pac;
	msg2host->pdc = inverter_info->inverter_meta_data.Pdc;

	return 1;
}





int convert_invmsg_to_asci(char *buff, inverter_msg2host_t* msg2host){
	int n;
	
	n = sprintf(buff, "%d,", msg2host->state_code); 
	buff += n;
	
	n = sprintf(buff, "%s,", msg2host->brand_name); 
	buff += n;	
	n = sprintf(buff, "%s,", msg2host->type_name); 
	buff += n;
	n = sprintf(buff, "%s,", msg2host->sn_name); 
	buff += n;
	n = sprintf(buff, "%d,", msg2host->total_hr_h); 
	buff += n;
	n = sprintf(buff, "%d,", msg2host->total_hr_l); 
	buff += n;
	n = sprintf(buff, "%d,", msg2host->total_min_h); 
	buff += n;
	n = sprintf(buff, "%d,", msg2host->total_min_l); 
	buff += n;	
	n = sprintf(buff, "%d,", msg2host->gen_H_h); 
	buff += n;
	n = sprintf(buff, "%d,", msg2host->gen_H_l); 
	buff += n;
	n = sprintf(buff, "%d,", msg2host->gen_L_h); 
	buff += n;
	n = sprintf(buff, "%d,", msg2host->gen_L_l); 
	buff += n;	
	n = sprintf(buff, "%d,", msg2host->pac_h); 
	buff += n;
	n = sprintf(buff, "%d,", msg2host->pac_l); 
	buff += n;
	n = sprintf(buff, "%d,", msg2host->pdc_h); 
	buff += n;
	n = sprintf(buff, "%d", msg2host->pdc_l); 
	buff += n;
	
	buff[0] = '\0';
	
	return 1; 
}

int convert_invmsg_to_asci_new(char *buff, inverter_msg2host_new_t* msg2host){
	int n;
	double tmp_gen, tmp_gen_h, tmp_gen_l;

	tmp_gen_h = (uint32_t)msg2host->total_gen_h;
	tmp_gen_h = tmp_gen_h * 1000;
	tmp_gen_l = (uint32_t)msg2host->total_gen_l;
	tmp_gen_l = tmp_gen_l /10;

	tmp_gen = tmp_gen_h + tmp_gen_l;
	
	n = sprintf(buff, "%d,", msg2host->state_code); 
	buff += n;
	n = sprintf(buff, "%.3d,", msg2host->inv_id); 
	buff += n;
	n = sprintf(buff, "%s,", msg2host->brand_name); 
	buff += n;	
	n = sprintf(buff, "%s,", msg2host->type_name); 
	buff += n;
	n = sprintf(buff, "%s,", msg2host->sn_name); 
	buff += n;
	n = sprintf(buff, "%d:", msg2host->total_hr); 
	buff += n;
	n = sprintf(buff, "%d:", msg2host->total_min); 
	buff += n;
	n = sprintf(buff, "%d,", msg2host->total_sec); 
	buff += n;
	n = sprintf(buff, "%.2f,", tmp_gen); 
	buff += n;	
	n = sprintf(buff, "%d,", msg2host->pac); 
	buff += n;
	n = sprintf(buff, "%d", msg2host->pdc); 
	buff += n;
	
	buff[0] = '\0';
	
	return 1; 
	
}

int isready(int fd)
{
	int rc;
  fd_set fds;
  struct timeval tv;

  FD_ZERO(&fds);
  FD_SET(fd,&fds);
  tv.tv_sec = 3;
  tv.tv_usec = 0;

  	rc = select(fd+1,&fds,NULL,NULL,&tv);

  if(rc < 0)
     return FD_ISSET(fd,&fds) ? 1 : 0;
  else if(rc==0)
  	return 0;
  else
  	return 0;
}

void switch_bytes(uint8_t* data_ptr, int len){
	uint8_t tmp_byte;
	int i;
	
	for(i=0; i < (len<<1); i=i+2){
		tmp_byte = data_ptr[i+0];
		data_ptr[i+0] = data_ptr[i+1];
		data_ptr[i+1] = tmp_byte;
	}
}


int open_port_RS485(char* device_node, int mode){
  int fd_rs485;
  struct termios options;

  if(!device_node)
  	return -1;

/*
  if(strcmp(device_node, "/dev/ttyUSB0")==0){
  	puts("force USB power On !");
  	system("devmem2 0x47401c60 b 0x01");
  }
*/	

  fd_rs485 = open(device_node, mode);

  if(fd_rs485 == -1)
    {
      perror("open_port: Unable to open:");
      return -1;
    }
    else
    {
      fcntl(fd_rs485, F_SETFL, 0);
    }
  
  tcgetattr(fd_rs485, &options);

  bzero(&options, sizeof(options));
    
  options.c_cflag = INVERTER_BAUDRATE |CLOCAL | CREAD;//(CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;//|= ~PARENB;//&= ~PARENB;
  options.c_cflag &= ~CSTOPB;//options.c_cflag &= ~CSTOPB;
  options.c_cflag |= CS8;
    
  options.c_iflag = IGNPAR;
  options.c_oflag &= ~OPOST;
  options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
  options.c_cc[VMIN] = 0;		// read() return after receiving VMIN of 
								// words, it works
  options.c_cc[VTIME] = 2;	// 4 works

  tcflush(fd_rs485, TCIFLUSH);
  tcsetattr(fd_rs485, TCSANOW, &options);
  
  return fd_rs485;
}

int self_json_pack_string_mid(char* string_p, char* name, char* value){

	if((string_p==NULL)||(name==NULL)||(value==NULL))
		return -1;

	sprintf(string_p, "  \"%s\": \"%s\",\n", name, value);
	return 1;
}

int self_json_pack_string_tail(char* string_p, char* name, char* value){

	if((string_p==NULL)||(name==NULL)||(value==NULL))
		return -1;

	sprintf(string_p, "  \"%s\": \"%s\"\n", name, value);
	return 1;
}

int list_add(eversolar_list_t** head, uint8_t id, uint8_t* sn){
	eversolar_list_t *current, *new_item;

	if(sn == NULL)
		return -1;

	// create an item
	new_item = malloc(sizeof(eversolar_list_t));

	// initial the body
	new_item->ID = id;
	memcpy(new_item->SN, sn, 18);
	new_item->SN[18] = '\0';
	new_item->aging_cnt = 0;
	new_item->next = NULL;

	current = *head;
	// check if empty
	if(current == NULL){
		*head = new_item;
		puts("add first item to list !!");
		return 1;
	}

	// check if there's already an item having this is
	while(current != NULL){
		if(current->ID == id){
			printf("add list already has this ID=%d\n", id);
			free(new_item);
			return -1;
		}
		// add new item to tail
		if(current->next == NULL){
			current->next = new_item;
			return 1;
		}

		// advance to next item
		current = current->next;
	}

	puts("add list failed !!");
	free(new_item);
	return -1;
}

int list_del(eversolar_list_t** head, uint8_t id_to_del){
	eversolar_list_t *current, *prev;

	if(*head == NULL){
		puts("in list_del, list is empty");
		return -1;
	}
	// special case
	current = *head;
	if(current->ID == id_to_del){
		if(current->next != NULL)
			*head = current->next;
		else
			*head = NULL;
		free(current);
		return 1;
	}else{
		prev = *head;
		current = current->next;	
	}

	// normal case
	while(current != NULL){
		if(current->ID == id_to_del){
			prev->next = current->next;
			free(current);
			puts("entry was removed");
			return 1;
		}else
			prev = current;

		current = current->next;
	}

	puts("No entry was deleted");
	return -1;
}

int list_del_all(eversolar_list_t** head){
	eversolar_list_t *current, *next;

	if(*head == NULL){
		puts("in list_del, list is empty");
		return -1;
	}
	
	current = *head;
	while(current != NULL){
		next = current->next;
		free(current);
		current = next;
	}

	*head = NULL;
	return 1;
}

int open_port_3G(char* device_node, int mode){
  int fd_rs485;
  struct termios options;

  fd_rs485 = open(device_node, mode);

  if(fd_rs485 == -1)
    {

      perror("open_port: Unable to open:");
    }
    else
    {
      fcntl(fd_rs485, F_SETFL, 0);
    }
  
  tcgetattr(fd_rs485, &options);

  bzero(&options, sizeof(options));
    
  options.c_cflag = B115200 |CLOCAL | CREAD;//(CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;//|= ~PARENB;//&= ~PARENB;
  options.c_cflag &= ~CSTOPB;//options.c_cflag &= ~CSTOPB;
  options.c_cflag |= CS8;
    
  options.c_iflag = IGNPAR;
  options.c_oflag &= ~OPOST;
  options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
  options.c_cc[VMIN] = 0;		// read() return after receiving VMIN of 
								// words, it works
  options.c_cc[VTIME] = 4;

  tcflush(fd_rs485, TCIFLUSH);
  tcsetattr(fd_rs485, TCSANOW, &options);
  
  return fd_rs485;
}

int set_sim5215_reset(char* device){

	int ttyO4_port, rsp_loc;
	char cmd_string[64], isError[6], rtn_string[128];

	bzero(isError, sizeof(isError));
	
	ttyO4_port = open_port_3G(device, O_RDWR | O_NDELAY);
	if(ttyO4_port < 0){
		puts("open port error !!");
		close(ttyO4_port);
		return -1;
	}

	puts("Send RESET command to 3G module !!");

	//sleep(1);

	strcpy(cmd_string, "AT+CRESET\r");

	rsp_loc = strlen(cmd_string) + 2;  // return exact the size of cmd + '\r' + '\n'

	write(ttyO4_port, cmd_string, strlen(cmd_string));
	
	//usleep(500000);

	sleep(20);

	read(ttyO4_port, rtn_string, sizeof(rtn_string));

	if(rtn_string[rsp_loc] == 'O'){
		puts("cmd Ok");
		close(ttyO4_port);
		return 1;
	}
	else if(rtn_string[rsp_loc] == 'E'){
		memcpy(isError, &rtn_string[rsp_loc], sizeof(isError)-1);
		
		if(strcmp(isError, "ERROR") == 0){
			puts("cmd Error");
			close(ttyO4_port);
			return -1;
		}
	}

	close(ttyO4_port);

	return -1;
}

int get_sim5215_at(char* device){

	int ttyO4_port, rsp_loc;
	char cmd_string[64], isError[6], rtn_string[16];

	bzero(isError, sizeof(isError));
	
	ttyO4_port = open_port_3G(device, O_RDWR | O_NDELAY);
	if(ttyO4_port < 0){
		puts("open port error !!");
		close(ttyO4_port);
		return -1;
	}

	puts("Send AT command!!");

	//sleep(1);

	strcpy(cmd_string, "AT\r");

	rsp_loc = strlen(cmd_string) + 2;  // return exact the size of cmd + '\r' + '\n'

	write(ttyO4_port, cmd_string, strlen(cmd_string));
	
	usleep(500000);

	read(ttyO4_port, rtn_string, sizeof(rtn_string));

	if(rtn_string[rsp_loc] == 'O'){
		puts("cmd Ok");
		close(ttyO4_port);
		return 1;
	}
	else if(rtn_string[rsp_loc] == 'E'){
		memcpy(isError, &rtn_string[rsp_loc], sizeof(isError)-1);
		
		if(strcmp(isError, "ERROR") == 0){
			puts("cmd Error");
			close(ttyO4_port);
			return -1;
		}
	}

	close(ttyO4_port);

	return -1;
}


int get_sim5215_op(char* device, char* rsp_string, int len){

	int ttyO4_port, rsp_loc;
	char cmd_string[64], rtn_string[128], isError[6];
	int i=0, tmp_size=0; // invIDs[256], i;

	bzero(rtn_string, sizeof(rtn_string));

	bzero(isError, sizeof(isError));

	ttyO4_port = open_port_3G(device, O_RDWR | O_NDELAY);
	if(ttyO4_port < 0){
		puts("open port error !!");
		close(ttyO4_port);
		return -1;
	}

	puts("READ AT_COPS !!");

	//sleep(1);

	//strcpy(cmd_string, "AT+COPS?\r");
	strcpy(cmd_string, "AT+COPS?\r");

	rsp_loc = strlen(cmd_string) + 2;  // return exact the size of cmd + '\r' + '\n'

	write(ttyO4_port, cmd_string, strlen(cmd_string));
	printf("in get_sim5215_op cmd out:%s--\n", cmd_string);

	usleep(500000);

	tmp_size = read(ttyO4_port, rtn_string, sizeof(rtn_string));
	printf("in get_sim5215_op cmd in #%d bytes:%s--\n", tmp_size, rtn_string);

	if(rtn_string[rsp_loc] == 'O')
		puts("cmd Ok");
	else if(rtn_string[rsp_loc] == 'E'){
		memcpy(isError, &rtn_string[rsp_loc], sizeof(isError)-1);
		
		if(strcmp(isError, "ERROR") == 0){
			puts("cmd Error");
			strcpy(rsp_string, "SIM_Error");
			close(ttyO4_port);
			return -1;
		}
	}

	rsp_loc += 12; //locate to the first char of "Far EasTone", means 'F'
	               // actual return is like the following --> +COPS: 0,0,"Far EasTone"

	// find the tail '"' char
	i=0;
	tmp_size=0;
	while(rtn_string[rsp_loc+i] != '"'){
		tmp_size ++;
		if(tmp_size >= 255){
			puts("Get Operator name error !!");
			strcpy(rsp_string,"No_Op");
			close(ttyO4_port);
			return -1;
		}
		i++;
	}
	// tmp_size is the real amount of characters

	// we might not have retrun size greater than chars + 1, 1 is \0
	if(tmp_size > len-1)
		tmp_size = len-1;

	rtn_string[rsp_loc+tmp_size] = '\0';

	//memcpy(rsp_string, &rtn_string[rsp_loc], tmp_size);	
	strcpy(rsp_string, &rtn_string[rsp_loc]);

	close(ttyO4_port);

	return tmp_size;
}

int get_sim5215_ccid(char* device, char* rsp_string, int len){

	int ttyO4_port, rsp_loc;
	char cmd_string[64], rtn_string[128], isError[6];
	int i=0, tmp_size=0; // invIDs[256], i;

	// stop pppd first
	//system("./ppp-stop");

	bzero(rtn_string, sizeof(rtn_string));
	//bzero(rsp_string, sizeof(rsp_string));
	bzero(isError, sizeof(isError));
	//ttyO4_port = open_port_3G("/dev/ttyO4", O_RDWR);
	ttyO4_port = open_port_3G(device, O_RDWR | O_NDELAY);
	if(ttyO4_port < 0){
		puts("open port error !!");
		close(ttyO4_port);
		return -1;
	}

	puts("READ AT_CCID !!");

	//sleep(1);

	strcpy(cmd_string, "AT+CCID\r");

	rsp_loc = strlen(cmd_string) + 2;  // return exact the size of cmd + '\r' + '\n'

	write(ttyO4_port, cmd_string, strlen(cmd_string));
	
	usleep(500000);

	read(ttyO4_port, rtn_string, sizeof(rtn_string));

	if(rtn_string[rsp_loc] == 'O')
		puts("cmd Ok");
	else if(rtn_string[rsp_loc] == 'E'){
		memcpy(isError, &rtn_string[rsp_loc], sizeof(isError)-1);
		
		if(strcmp(isError, "ERROR") == 0){
			puts("cmd Error");
			strcpy(rsp_string, "SIM_Error");
			close(ttyO4_port);
			return -1;
		}
	}

	rsp_loc += 8; 

	// find the tail '"' char
	i=0;

	while(rtn_string[rsp_loc+i] != '"'){
		tmp_size ++;
		if(tmp_size >= 255){
			puts("Get CCID name error !!");
			close(ttyO4_port);
			return -1;
		}
		i++;
	}
	// tmp_size is the real amount of characters

	// we might not have retrun size greater than chars + 1, 1 is \0
	if(tmp_size > len-1)
		tmp_size = len-1;

	rtn_string[rsp_loc+tmp_size] = '\0';

	//memcpy(rsp_string, &rtn_string[rsp_loc], tmp_size);	
	strcpy(rsp_string, &rtn_string[rsp_loc]);

	close(ttyO4_port);

	return tmp_size;
}

int get_sim5215_rssi(char* device, char* rsp_string, int len){

	int ttyO4_port, rsp_loc;
	char cmd_string[64], rtn_string[128], isError[6];
	int i=0, tmp_size=0; // invIDs[256], i;

	bzero(rtn_string, sizeof(rtn_string));

	bzero(isError, sizeof(isError));

	ttyO4_port = open_port_3G(device, O_RDWR | O_NDELAY);
	if(ttyO4_port < 0){
		puts("open port error !!");
		close(ttyO4_port);
		return -1;
	}

	puts("READ AT_CSQ !!");

	//sleep(1);

	strcpy(cmd_string, "AT+CSQ\r");

	rsp_loc = strlen(cmd_string) + 2;  // return exact the size of cmd + '\r' + '\n'

	write(ttyO4_port, cmd_string, strlen(cmd_string));
	
	usleep(500000);

	read(ttyO4_port, rtn_string, sizeof(rtn_string));

	i=0;
	if(rtn_string[rsp_loc] == 'O')
		puts("cmd Ok");
	else if(rtn_string[rsp_loc] == 'E'){
		memcpy(isError, &rtn_string[rsp_loc], sizeof(isError)-1);
		
		if(strcmp(isError, "ERROR") == 0){
			puts("cmd Error");
			strcpy(rsp_string, "SIM_Error");
			close(ttyO4_port);
			return -1;
		}
	}

	rsp_loc += 6;

	// find the tail '"' char
	i=0;
	while(rtn_string[rsp_loc+i] != ','){
		tmp_size ++;
		if(tmp_size >= 255){
			puts("Get Operator name error !!");
			close(ttyO4_port);
			return -1;
		}
		i++;
	}
	// tmp_size is the real amount of characters

	// we might not have retrun size greater than chars + 1, 1 is \0
	if(tmp_size > len-1)
		tmp_size = len-1;

	// rsp_loc is pointing to the first character
	rtn_string[rsp_loc+tmp_size] = '\0';

	//memcpy(rsp_string, &rtn_string[rsp_loc], tmp_size);	
	strcpy(rsp_string, &rtn_string[rsp_loc]);

	close(ttyO4_port);

	return tmp_size;
}

int get_sim5215_info(char* device, char* rsp_string, int len){
	int rtn_size, return_size;
	char rsp_op[64], rsp_ccid[64], rsp_rssi[64], show_string[128];
	int rssi=0;

	usleep(500000);

	if(get_sim5215_at(device) > 0)
		puts("3G module ok !!");
	else{
		puts("3G module no response !!");
		return -1;
	}

	bzero(rsp_op, 64);
	rtn_size = get_sim5215_op(device, rsp_op, sizeof(rsp_op));
	printf("op cmd, We get #%d bytes, string=%s\n", rtn_size, rsp_op);

	bzero(rsp_ccid, 64);
	rtn_size = get_sim5215_ccid(device, rsp_ccid, sizeof(rsp_ccid));
	printf("ccid cmd, We get #%d bytes, string=%s\n", rtn_size, rsp_ccid);

	bzero(rsp_rssi, 64);
	rtn_size = get_sim5215_rssi(device, rsp_rssi, sizeof(rsp_rssi));
	printf("rssi cmd, We get #%d bytes, string=%s\n", rtn_size, rsp_rssi);
	rssi = atoi(rsp_rssi);

	if (rssi == 0)
		strcpy(rsp_rssi,"NoLink");
	else if(rssi < 10)
		strcpy(rsp_rssi,"Marginal");
	else if(rssi < 15)
		strcpy(rsp_rssi,"OK");
	else if(rssi < 20)
		strcpy(rsp_rssi,"Good");
	else if(rssi < 30)
		strcpy(rsp_rssi,"Excellent");
	else
		strcpy(rsp_rssi,"RSSI_ERR");

	rtn_size = sprintf(show_string, "Tele=%s,Ccid=%s,Rssi=%s", rsp_op, rsp_ccid, rsp_rssi);

	if(rtn_size > 127)
		return -1;

	// rtn_size is only the amount of char, but show_string's tail contain an extra '\0'
	if(rtn_size + 1 > len)
		return_size = len;
	else 
		return_size = rtn_size + 1;

	show_string[return_size - 1] = '\0';

	memcpy(rsp_string, show_string, return_size);

	//sleep(2);

	return return_size;
}

int read_3g_info_file(char* fileName, char* rtn_info, int len){
	FILE* file2read;
	char * line = NULL;
    size_t getlen = 0;
    ssize_t read;

	file2read = fopen(fileName, "r");
	if (file2read == NULL)
		return -1;
    
    if((read = getline(&line, &getlen, file2read)) != -1){
        //line[read-1]='\0';

        if((read>1) && (read < len))
        	strcpy(rtn_info, line);
    }

    if (line)
        free(line);

	fclose(file2read);
	return 1;

}

int write_3g_info_file(char* fileName){
	FILE* file2write;
	int rtn_3G_exist = 0;

	// open file
	file2write = fopen(fileName, "w");
	if (file2write==NULL){
		printf("Open fine=%s error !!\n", fileName);
		return -1;
	}

	// check 3G network
	rtn_3G_exist = get_sim5215_info(DEV_3G, info_3G, sizeof(info_3G));
	if(rtn_3G_exist > 0) {
		// write to file
		info_3G[strlen(info_3G)]='\0';	// a little bit dummy ??
		fwrite(info_3G, sizeof(char), strlen(info_3G), file2write);
	}else{
		puts("get 3G info failed !!");
		strcpy(info_3G,"Tele=No3G,Ccid=No3G,Rssi=No3G");
		fwrite(info_3G, sizeof(char), strlen(info_3G), file2write);
		fclose(file2write);
		system("sync");
		return -1;
	}
	
	// close
	fclose(file2write);

	system("sync");

	return 1;
}

int clear_serial_rx_buffer(int port){
	int total_byte=0, bytes;
	char rcv_buff[32];
	int i;
	bytes = 1;
	usleep(400000);
	
	while(bytes > 0){
		bytes = read(port, (char*)rcv_buff, 32);
		if(bytes <0){
			puts("Clear serial rx buffer Error !!");
			return -1;
		}
		total_byte +=bytes;
		for(i=0;i<bytes;i++)
			printf("%02x ",rcv_buff[i]);
	}
	puts(" ");
	printf("Clear total %d bytes on serial port %d\n", total_byte, port);
	return total_byte;
}


char* string_trim(char* str) {
        char *str_last, *str_cur;
        if (str == NULL)
                return NULL;
        // front
        for (; *str == 0x20 || *str == '\t' || *str == '\n'; ++str)
                ;

        // behind
        for (str_last = str_cur = str; *str_cur != '\0'; ++str_cur) {
                if (*str_cur != 0x20 && *str_cur != '\t' && *str_cur != '\n')
                        str_last = str_cur;
        }
        *++str_last = 0;
        return str;
}

int acquire_unique_name(char * path2xml, char * uniqueName){
    xmlDoc         *document;
    xmlNode        *root, *first_child, *node;
    xmlNode        *node1, *node2, *nodeOUI, *nodeSerial;
    char           *filename;
    char          OUI_string[64], Serial_string[64], rtn_string[128];
	// read xml

	// parse xml

	// compose return string

	// return

    if (path2xml == NULL) {
        puts("input xml file is illegal");
        return -1;
    }
    filename = path2xml;

    document = xmlReadFile(filename, NULL, 0);
    root = xmlDocGetRootElement(document);
    fprintf(stdout, "Root is <%s> (%i)\n", root->name, root->type);
    first_child = root->children;
    for (node = first_child; node; node = node->next) {
        fprintf(stdout, "\t Child is <%s> (%i)\n", node->name, node->type);
      if(strcmp((char*)node->name, "Device")==0){
        puts("Enter Device level");
        for(node1 = node->children; node1; node1 = node1->next){
          fprintf(stdout, "\t Device Child is <%s> (%i)\n", node1->name, node1->type);
          if(strcmp((char*)node1->name, "DeviceInfo")==0){
            puts("Enter DeviceInfo level");
            for(node2 = node1->children; node2; node2 = node2->next){
              fprintf(stdout, "\t DeviceInfo Child is <%s> (%i)\n", node2->name, node2->type);

              if(strcmp((char*)node2->name, "ManufacturerOUI")==0){
                nodeOUI = node2->children;
                fprintf(stdout, "\t OUI content is <%s>\n", nodeOUI->content);
                nodeOUI->content = (xmlChar*)string_trim((char*)(nodeOUI->content));
                strcpy(OUI_string, (char*)(nodeOUI->content));
              }
              if(strcmp((char*)node2->name, "SerialNumber")==0){
                nodeSerial = node2->children;
                fprintf(stdout, "\t Serial content is <%s>\n", nodeSerial->content);
                nodeSerial->content = (xmlChar*)string_trim((char*)(nodeSerial->content));
                strcpy(Serial_string, (char*)(nodeSerial->content));
              }
            }
          }
        }
      }
    }
    fprintf(stdout, "...\n");
    
    sprintf(rtn_string, "%s-%s", OUI_string, Serial_string);
    strcpy(uniqueName, rtn_string);

    //xmlFreeDoc(document);
    return 1;
}


int change_unique_name_my(char * path2xml, char * uniqueName){
	int nb=0;
	char path_filename[128];

	bzero(path_filename, 128);

	xml_node* my_Root = xml_load_doc(path2xml, "TR106_BaselineProfile.xml");
	xml_node** ans = xml_xpath(my_Root, "//DeviceInfo/SerialNumber", &nb);

	if(nb){
    	puts("found unique ID!!");
    	xml_set_content(ans[0], uniqueName);
    	
    	sprintf(path_filename, "%sTR106_BaselineProfile.xml", path2xml);

    	xml_release(ans);
	    //xml_save_doc(my_Root, "TR106_BaselineProfile.xml");
	    xml_save_doc(my_Root, path_filename);
    	xml_close(my_Root);
    	return 1;
    }else{
    	puts("No unique ID found, write error!!");
    	xml_release(ans);
    	xml_close(my_Root);
    	return -1;
	}
    
    return -1;
}


int change_unique_name(char * path2xml, char * uniqueName){
    xmlDoc         *document;
    xmlNode        *root, *first_child, *node;
    xmlNode        *node1, *node2, *nodeOUI, *nodeSerial;
    char           *filename;
    char          OUI_string[64], Serial_string[64], rtn_string[128];

    if (path2xml == NULL) {
        puts("input xml file is illegal");
        return -1;
    }
    filename = path2xml;

    document = xmlReadFile(filename, NULL, 0);
    root = xmlDocGetRootElement(document);
    fprintf(stdout, "Root is <%s> (%i)\n", root->name, root->type);
    first_child = root->children;
    for (node = first_child; node; node = node->next) {
        fprintf(stdout, "\t Child is <%s> (%i)\n", node->name, node->type);
      if(strcmp((char*)node->name, "Device")==0){
        puts("Enter Device level");
        for(node1 = node->children; node1; node1 = node1->next){
          fprintf(stdout, "\t Device Child is <%s> (%i)\n", node1->name, node1->type);
          if(strcmp((char*)node1->name, "DeviceInfo")==0){
            puts("Enter DeviceInfo level");
            for(node2 = node1->children; node2; node2 = node2->next){
              fprintf(stdout, "\t DeviceInfo Child is <%s> (%i)\n", node2->name, node2->type);

              if(strcmp((char*)node2->name, "ManufacturerOUI")==0){
                nodeOUI = node2->children;
                fprintf(stdout, "\t OUI content is <%s>\n", nodeOUI->content);
                nodeOUI->content = (xmlChar*)string_trim((char*)(nodeOUI->content));
                strcpy(OUI_string, (char*)(nodeOUI->content));
              }
              if(strcmp((char*)node2->name, "SerialNumber")==0){
                nodeSerial = node2->children;
                fprintf(stdout, "\t Serial content is <%s>\n", nodeSerial->content);
                nodeSerial->content = (xmlChar*)string_trim((char*)(nodeSerial->content));
                strcpy(Serial_string, (char*)(nodeSerial->content));
              }
            }
          }
        }
      }
    }
    fprintf(stdout, "...\n");
    
    sprintf(rtn_string, "%s-%s", OUI_string, Serial_string);
    strcpy(uniqueName, rtn_string);

    //xmlFreeDoc(document);
    return 1;
}

uint16_t append_FCS_add(uint8_t * data, int len){
	uint16_t result = 0;
	int i;

	if(data==NULL)
		return -1;

	if(len > 32) // abortrary value
		return -1;

	for(i=0; i<len; i++)
		result += data[i];

	return result;
}

#define PREFIX_GW 0xff
#define GW_CMD_REQ 0x01
#define GW_CMD_RSP 0x11

int create_gateway_redundant_req(gw_sync_pkt_t* packet, char * path2xml){
	char tmp_time[15];
	char tmp_out_name[32];

	bzero(tmp_time, sizeof(tmp_time));

	syscall_getRsp("date +%Y%m%d%H%M%S", tmp_time);
	tmp_time[14] = '\0';
	
	packet->prefix = PREFIX_GW;
	strcpy(packet->timestamp, tmp_time);
	
	acquire_unique_name(path2xml, tmp_out_name);

	strcpy(packet->out_serial, tmp_out_name);

	packet->cmd_rsp = GW_CMD_REQ;

	packet->FCS = append_FCS_add((uint8_t*)packet, sizeof(gw_sync_pkt_t)-2 );

	return 1;
}

int create_gateway_redundant_rsp(gw_sync_pkt_t* packet, char * path2xml){
	char tmp_time[15];
	char tmp_out_name[32];

	bzero(tmp_time, sizeof(tmp_time));

	syscall_getRsp("date +%Y%m%d%H%M%S", tmp_time);
	tmp_time[14] = '\0';
	
	packet->prefix = PREFIX_GW;
	strcpy(packet->timestamp, tmp_time);
	
	acquire_unique_name(path2xml, tmp_out_name);

	strcpy(packet->out_serial, tmp_out_name);

	packet->cmd_rsp = GW_CMD_RSP;

	packet->FCS = append_FCS_add((uint8_t*)packet, sizeof(gw_sync_pkt_t)-2 );

	return 1;
}


#define TOKEN_EQ(t, tok_start, tok_end, tok_type) \
        ((t).start == tok_start \
         && (t).end == tok_end  \
         && (t).type == (tok_type))

#define TOKEN_STRING(js, t, s) \
        (strncmp(js+(t).start, s, (t).end - (t).start) == 0 \
         && (int)strlen(s) == (t).end - (t).start)

#define TOKEN_PRINT(t) \
        printf("start: %d, end: %d, type: %d, size: %d\n", \
                        (t).start, (t).end, (t).type, (t).size)


#define JSMN_STRICT

/**
 * Allocates a fresh unused token from the token pull.
 */
static jsmntok_t *jsmn_alloc_token(jsmn_parser *parser, 
		jsmntok_t *tokens, size_t num_tokens) {
	jsmntok_t *tok;
	if (parser->toknext >= num_tokens) {
		return NULL;
	}
	tok = &tokens[parser->toknext++];
	tok->start = tok->end = -1;
	tok->size = 0;
#ifdef JSMN_PARENT_LINKS
	tok->parent = -1;
#endif
	return tok;
}

/**
 * Fills token type and boundaries.
 */
static void jsmn_fill_token(jsmntok_t *token, jsmntype_t type, 
                            int start, int end) {
	token->type = type;
	token->start = start;
	token->end = end;
	token->size = 0;
}

/**
 * Fills next available token with JSON primitive.
 */
static jsmnerr_t jsmn_parse_primitive(jsmn_parser *parser, const char *js,
		size_t len, jsmntok_t *tokens, size_t num_tokens) {
	jsmntok_t *token;
	int start;

	start = parser->pos;

	for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
		switch (js[parser->pos]) {
#ifndef JSMN_STRICT
			/* In strict mode primitive must be followed by "," or "}" or "]" */
			case ':':
#endif
			case '\t' : case '\r' : case '\n' : case ' ' :
			case ','  : case ']'  : case '}' :
				goto found;
		}
		if (js[parser->pos] < 32 || js[parser->pos] >= 127) {
			parser->pos = start;
			return JSMN_ERROR_INVAL;
		}
	}
#ifdef JSMN_STRICT
	/* In strict mode primitive must be followed by a comma/object/array */
	parser->pos = start;
	return JSMN_ERROR_PART;
#endif

found:
	if (tokens == NULL) {
		parser->pos--;
		return 0;
	}
	token = jsmn_alloc_token(parser, tokens, num_tokens);
	if (token == NULL) {
		parser->pos = start;
		return JSMN_ERROR_NOMEM;
	}
	jsmn_fill_token(token, JSMN_PRIMITIVE, start, parser->pos);
#ifdef JSMN_PARENT_LINKS
	token->parent = parser->toksuper;
#endif
	parser->pos--;
	return 0;
}

/**
 * Filsl next token with JSON string.
 */
static jsmnerr_t jsmn_parse_string(jsmn_parser *parser, const char *js,
		size_t len, jsmntok_t *tokens, size_t num_tokens) {
	jsmntok_t *token;

	int start = parser->pos;

	parser->pos++;

	/* Skip starting quote */
	for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
		char c = js[parser->pos];

		/* Quote: end of string */
		if (c == '\"') {
			if (tokens == NULL) {
				return 0;
			}
			token = jsmn_alloc_token(parser, tokens, num_tokens);
			if (token == NULL) {
				parser->pos = start;
				return JSMN_ERROR_NOMEM;
			}
			jsmn_fill_token(token, JSMN_STRING, start+1, parser->pos);
#ifdef JSMN_PARENT_LINKS
			token->parent = parser->toksuper;
#endif
			return 0;
		}

		/* Backslash: Quoted symbol expected */
		if (c == '\\') {
			parser->pos++;
			switch (js[parser->pos]) {
				/* Allowed escaped symbols */
				case '\"': case '/' : case '\\' : case 'b' :
				case 'f' : case 'r' : case 'n'  : case 't' :
					break;
				/* Allows escaped symbol \uXXXX */
				case 'u':
					parser->pos++;
					int i = 0;
					for(; i < 4 && js[parser->pos] != '\0'; i++) {
						/* If it isn't a hex character we have an error */
						if(!((js[parser->pos] >= 48 && js[parser->pos] <= 57) || /* 0-9 */
									(js[parser->pos] >= 65 && js[parser->pos] <= 70) || /* A-F */
									(js[parser->pos] >= 97 && js[parser->pos] <= 102))) { /* a-f */
							parser->pos = start;
							return JSMN_ERROR_INVAL;
						}
						parser->pos++;
					}
					parser->pos--;
					break;
				/* Unexpected symbol */
				default:
					parser->pos = start;
					return JSMN_ERROR_INVAL;
			}
		}
	}
	parser->pos = start;
	return JSMN_ERROR_PART;
}

/**
 * Parse JSON string and fill tokens.
 */
jsmnerr_t jsmn_parse(jsmn_parser *parser, const char *js, size_t len,
		jsmntok_t *tokens, unsigned int num_tokens) {
	jsmnerr_t r;
	int i;
	jsmntok_t *token;
	int count = 0;

	for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
		char c;
		jsmntype_t type;

		c = js[parser->pos];
		switch (c) {
			case '{': case '[':
				count++;
				if (tokens == NULL) {
					break;
				}
				token = jsmn_alloc_token(parser, tokens, num_tokens);
				if (token == NULL)
					return JSMN_ERROR_NOMEM;
				if (parser->toksuper != -1) {
					tokens[parser->toksuper].size++;
#ifdef JSMN_PARENT_LINKS
					token->parent = parser->toksuper;
#endif
				}
				token->type = (c == '{' ? JSMN_OBJECT : JSMN_ARRAY);
				token->start = parser->pos;
				parser->toksuper = parser->toknext - 1;
				break;
			case '}': case ']':
				if (tokens == NULL)
					break;
				type = (c == '}' ? JSMN_OBJECT : JSMN_ARRAY);
#ifdef JSMN_PARENT_LINKS
				if (parser->toknext < 1) {
					return JSMN_ERROR_INVAL;
				}
				token = &tokens[parser->toknext - 1];
				for (;;) {
					if (token->start != -1 && token->end == -1) {
						if (token->type != type) {
							return JSMN_ERROR_INVAL;
						}
						token->end = parser->pos + 1;
						parser->toksuper = token->parent;
						break;
					}
					if (token->parent == -1) {
						break;
					}
					token = &tokens[token->parent];
				}
#else
				for (i = parser->toknext - 1; i >= 0; i--) {
					token = &tokens[i];
					if (token->start != -1 && token->end == -1) {
						if (token->type != type) {
							return JSMN_ERROR_INVAL;
						}
						parser->toksuper = -1;
						token->end = parser->pos + 1;
						break;
					}
				}
				/* Error if unmatched closing bracket */
				if (i == -1) return JSMN_ERROR_INVAL;
				for (; i >= 0; i--) {
					token = &tokens[i];
					if (token->start != -1 && token->end == -1) {
						parser->toksuper = i;
						break;
					}
				}
#endif
				break;
			case '\"':
				r = jsmn_parse_string(parser, js, len, tokens, num_tokens);
				if (r < 0) return r;
				count++;
				if (parser->toksuper != -1 && tokens != NULL)
					tokens[parser->toksuper].size++;
				break;
			case '\t' : case '\r' : case '\n' : case ':' : case ',': case ' ': 
				break;
#ifdef JSMN_STRICT
			/* In strict mode primitives are: numbers and booleans */
			case '-': case '0': case '1' : case '2': case '3' : case '4':
			case '5': case '6': case '7' : case '8': case '9':
			case 't': case 'f': case 'n' :
#else
			/* In non-strict mode every unquoted value is a primitive */
			default:
#endif
				r = jsmn_parse_primitive(parser, js, len, tokens, num_tokens);
				if (r < 0) return r;
				count++;
				if (parser->toksuper != -1 && tokens != NULL)
					tokens[parser->toksuper].size++;
				break;

#ifdef JSMN_STRICT
			/* Unexpected char in strict mode */
			default:
				return JSMN_ERROR_INVAL;
#endif
		}
	}

	for (i = parser->toknext - 1; i >= 0; i--) {
		/* Unmatched opened object or array */
		if (tokens[i].start != -1 && tokens[i].end == -1) {
			return JSMN_ERROR_PART;
		}
	}

	return count;
}

/**
 * Creates a new parser based over a given  buffer with an array of tokens 
 * available.
 */
void jsmn_init(jsmn_parser *parser) {
	parser->pos = 0;
	parser->toknext = 0;
	parser->toksuper = -1;
}

jsmntype_t get_value_by_key(char* js, jsmntok_t* tokens, int token_num, char* string, char* rtn_string){
  int i;
  int max_size;

  printf("Input=%s\n", string);

  max_size = tokens[0].size;
  if(max_size>token_num)
    return JSMN_NONE;

  for(i=1;i<max_size;i++){
  	TOKEN_PRINT(tokens[i]);
    if(TOKEN_STRING(js, tokens[i], string)){
      memcpy(rtn_string, js+tokens[i+1].start, tokens[i+1].end - tokens[i+1].start);
      return tokens[i+1].type;
    }
  }
  puts("can't find");
  strcpy(rtn_string, "");
  return JSMN_NONE;
}


int network_3G_start(void){
  	//char info_3G[64];
  	//int rssi;
  	int rtn_3G_exist;
  	// reset 3G module
  	if(set_sim5215_reset(DEV_3G) > 0)
		puts("3G module ok !!");
	else{
		puts("3G module no response !!");
		return -1;
	}
	// wait for 15 seconds to get 3g module ready
	//sleep(15);  // move this waiting time into set_sim5125_reset()

	rtn_3G_exist = write_3g_info_file(FILE_3G_INFO);
	/*
	rtn_3G_exist = get_sim5215_info(DEV_3G, info_3G, sizeof(info_3G));
	rssi =0;
	printf("What we got in info_3g=%s \n", info_3G);
	sscanf(info_3G, "Rssi=%d", &rssi);
	printf("We get rssi=%d\n", rssi);
	*/
	puts("start 3g network !!");
	if(rtn_3G_exist > 0) { // 3G does response, but not sure whether we can connect
		system("pppd call wcdma &");
		system("sync");
	    sleep(7);
	    system("route del default");
	    system("route del default; route add default gw 10.64.64.64 ppp0");
	    system("echo nameserver 8.8.8.8 > /etc/resolv.conf");
	    system("echo nameserver 168.95.192.1 >> /etc/resolv.conf");
	    system("echo nameserver 168.95.1.1 >> /etc/resolv.conf");
	}
	return 1;
}

int network_eth0_start(void){
	char rtnStr[24];
	char cmd[1024];

	bzero(cmd, 1024);
	bzero(rtnStr, 24);
	sprintf(cmd, "ifconfig |awk '/eth/{print $1}'");
	syscall_getRsp(cmd, rtnStr);
	//printf("Rtn=%s \n", rtnStr);
	rtnStr[4] = '\0';
	if(strcmp(rtnStr, "eth0")==0){
		puts("try to use eth0 as Egress interface");
		system("dhclient eth0 &");
		sleep(5);
	}
	return 1;
}

int network_wifi_start(void){
	char rtnStr[24];
	char cmd[1024];

	bzero(cmd, 1024);
	bzero(rtnStr, 24);
	sprintf(cmd, "ifconfig |awk '/wlan/{print $1}'");
	syscall_getRsp(cmd, rtnStr);
	//printf("Rtn=%s \n", rtnStr);
	rtnStr[5] = '\0';
	if(strcmp(rtnStr, "wlan0")==0){
		puts("try to use wlan0 as Egress interface");
	}	
	return 1;
}

int network_3G_stop(void){
	//char rtnStr[24];
	//char cmd[1024];
	//char rtn_str[64];
	//int rtn_num;
	// 3.0. kill /var/lock/LCK..ttyO4 if existed
	system("sudo rm /var/lock/LCK..ttyO*");
	// 3.1. start wcdma_empty, solve the async issue
	system("poff -a");
	sleep(2);
	system("pppd call wcdma_empty &");
	//system("sync");
	sleep(5);	// 5 seconds is a must !!
	// 3.2. remove all pppd
	system("poff -a");
	sleep(8);  // 8 seconds is a must for wcdma_empty to stop !!
	/*
	while(1){
	    // find pppd process
	    bzero(rtn_str, 64);
	    bzero(cmd, 1024);
	    bzero(rtnStr, 24);
	    rtn_num=0;
	    sprintf(cmd, "ps aux |awk '/pppd call wcdma/{print $2 " " $11}'|sed '2,10d'");
	    syscall_getRsp(cmd, rtnStr);
	    sscanf(rtnStr, "%d %s", &rtn_num, rtn_str);
	    printf("We get rtn_str=%s, rtn_num=%d\n", rtn_str, rtn_num);
	    // if process is pppd, kill it !!
	    if(strcmp(rtn_str,"pppd")==0){
	    	bzero(cmd, 1024);
	    	printf("Found daemon=%s, pid=%d, kill it !!\n", rtn_str, rtn_num);
	    	sprintf(cmd, "kill %d", rtn_num);
	    	system(cmd);
	    }else{
	    	puts("No pppd daemon found so far !!");
	    	break;
	    }
	    sleep(4);
	}
	*/
	return 1;
}

int network_eth0_stop(void){
	char rtnStr[24];
	char cmd[1024];
	char rtn_str[64];
	int rtn_num;	
	while(1){
		bzero(cmd, 1024);
		bzero(rtnStr, 24);
		bzero(rtn_str, sizeof(rtn_str));
		rtn_num=0;
		sprintf(cmd, "ps aux |awk '/dhclient/{print $2 " " $11}'|sed '2,10d'");
		syscall_getRsp(cmd, rtnStr);
		sscanf(rtnStr, "%d %s", &rtn_num, rtn_str);
		//printf("Rtn=%s \n", rtnStr);
		rtn_str[8]='\0';
		if(strcmp(rtn_str,"dhclient")==0){
    		puts("dhclient exist !! kill it!!");
    		bzero(cmd,1024);
    		sprintf(cmd, "kill %d", rtn_num);
    		system(cmd); 	
    	}else{
    		puts("No dhclient so far !!");
    		break;
    	}
    }

    return 1;
}

int init_USB_LCD(char * device){
int fd;
char cmd_buf[8];
        if(device == NULL)
                return -1;

        fd = open(device, O_RDWR);
        if(fd<0){
                printf("USB_LCD %s not exist !!", device);
                return -1;
        }

        // auto scroll off
        cmd_buf[0]=LCD_CMD_PRE;
        cmd_buf[1]=LCD_CMD_ASCROFF;
        write(fd, cmd_buf, 2);

        // clear all
        cmd_buf[0]=LCD_CMD_PRE;
        cmd_buf[1]=LCD_CMD_CLRSCR;
        write(fd, cmd_buf, 2);

        close(fd);

        printf("USB_LCD %s init OK !!", device);
        return 1;
}

int ctrl_USB_LCD(char* device, int cmd){
int fd;
char cmd_buf[8];
        if(device == NULL)
                return -1;

        fd = open(device, O_RDWR);
        if(fd<0){
                printf("USB_LCD %s not exist !!", device);
                return -1;
        }

        switch(cmd){
                case LCD_CMD_CLRSCR:
                        cmd_buf[0]=LCD_CMD_PRE;
                        cmd_buf[1]=LCD_CMD_CLRSCR;
                        write(fd, cmd_buf, 2);
                break;
                default:;
        }

        close(fd);
        return 1;
}

int string_USB_LCD(char* device, char* string, int line){
int fd;
char cmd_buf[8];
int allow_size=0;
        if(device == NULL)
                return -1;

        fd = open(device, O_RDWR);
        if(fd<0){
                printf("USB_LCD %s not exist !!", device);
                return -1;
        }
        if(line != 2)
                line = 1;

        // set curser
        cmd_buf[0]=LCD_CMD_PRE;
        cmd_buf[1]=LCD_CMD_SETCUR;
        cmd_buf[2]=0x01;
        cmd_buf[3]=line;
        write(fd, cmd_buf, 4);

        // clear that line
        write(fd, "                ", 16);

        // set curser
        cmd_buf[0]=LCD_CMD_PRE;
        cmd_buf[1]=LCD_CMD_SETCUR;
        cmd_buf[2]=0x01;
        cmd_buf[3]=line;
        write(fd, cmd_buf, 4);

        // organize string
        allow_size = strlen(string);
        if(allow_size > 16)
                allow_size = 16;

        write(fd, string, allow_size);

        close(fd);
        return 1;
}

int network_link_check(void){
  	int ping_qualify, ping_count;

  	char rtnStr[24];
  	char cmd[1024];

  	bzero(rtnStr, 24);
  	sprintf(cmd, "ping -c %d %s | grep 'received' | awk -F',' '{ print $2 }' | awk '{ print $1 }'", TOTAL_PING_COUNT, IP_TO_PROBE); 
  	syscall_getRsp(cmd, rtnStr);
  	ping_count = atoi(rtnStr);
  	ping_qualify = TOTAL_PING_COUNT >> 1; // make half of the total as the qualify standard 

  	if(ping_count > ping_qualify){
   		puts(" network OK !!");

   		// check interface
		bzero(cmd, 1024);
		bzero(rtnStr, 24);
		sprintf(cmd, "route -n|awk '/^0.0.0.0/{print $8}'");
		syscall_getRsp(cmd, rtnStr);
		//printf("Rtn=%s \n", rtnStr);
		rtnStr[4] = '\0';
		if(strcmp(rtnStr,"ppp0")==0){
    		puts("Egress interface = 3G");
    		return NETWORK_LINK_3G;
    		//system("./setLed1Blink3.sh &");
    	}else if(strcmp(rtnStr,"eth0")==0){
    		puts("Egress interface = wired");
    		return NETWORK_LINK_WIRED;
    		//system("./setLed1Blink1.sh &");
    	}else if(strcmp(rtnStr,"wlan")==0){
    		puts("Egress interface = wifi");
    		return NETWORK_LINK_WIFI;
    		//system("./setLed1Blink2.sh &");
    	}else{
    		puts("Egress interface = unknown!!");
    		return NETWORK_LINK_UNKNOWN;
    		//system("./setLed1Bright.sh &");
    	}

  	}
  	else {
    	puts(" Network disconnected !!! "); 
    	return NETWORK_LINK_LOST;
    }
    return NETWORK_LINK_UNKNOWN;
}
