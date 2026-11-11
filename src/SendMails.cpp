// This task manages the mails to be sent in case of alarm:
// 0 - Over-pressure
// 1 - Chlorine level low
// 2 - Acid level low
// 3 - Chlore pump max uptime 
// 4 - Ph pump max uptime
// The message ID (1-5) and the associated value is received in a queue which is 
// polled every 10 seconds.
// This task can take a lot of time to execute due to SMTP servers delay, reason why
// it is let running asynchronously from the other loops, without watch-dog.

#include <Arduino.h>                // Arduino framework
#include "Config.h"
#include "PoolMaster.h"
#include <ESP_Mail_Client.h>

SMTPSession smtp;
Session_Config config;
SMTP_Message message;

void smtpCallback(SMTP_Status);
bool SMTP_Connect(void);

void SendMails(void *pvParameters)
{  
  MailClient.networkReconnect(pdFALSE);
  #ifndef SILENT_MODE
    smtp.debug(1);
  #endif
  smtp.callback(smtpCallback);
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_LOGIN;
  config.login.password = AUTHOR_PASSWORD;
  config.login.user_domain = "127.0.0.1";

  message.sender.name = F("PoolMaster");
  message.sender.email = AUTHOR_EMAIL;
  message.subject = F("PoolMaster Event");
  message.addRecipient(F("Home"), RECIPIENT_EMAIL);
  message.text.charSet = "us-ascii";
  message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;
  message.priority = esp_mail_smtp_priority_low;
  message.response.notify = esp_mail_smtp_notify_success | esp_mail_smtp_notify_failure | esp_mail_smtp_notify_delay;

  char texte[80] = "";
  struct __attribute__((packed)) MsgStruct {uint8_t Alert_ID;double Alert_Value;}; // Force padding to 9 bytes instead of 16
  MsgStruct msg = {0,0.};

  for(;;)
  {
    if (uxQueueMessagesWaiting(queueMails) != 0)
    {  
      xQueueReceive(queueMails,&msg,0);
      switch (msg.Alert_ID)
      {
      case 0:
        sprintf(texte,"Water pressure alert: %4.2f bar",msg.Alert_Value);
        break;
      case 1:
        sprintf(texte,"Chlorine level LOW: %3.0f %%",msg.Alert_Value);
        break;
      case 2:
        sprintf(texte,"Acid level LOW: %3.0f %%",msg.Alert_Value);
        break;
      case 3:
        sprintf(texte,"Chlorine pump uptime: %2.0fmn",msg.Alert_Value);
        break;
      case 4:
        sprintf(texte,"Acid pump uptime: %2.0fmn",msg.Alert_Value);
        break;         
      }
      message.text.content = texte;
      if(WiFi.status() == WL_CONNECTED){
        if(SMTP_Connect()){   
          if(!MailClient.sendMail(&smtp, &message))
            Debug.print(DBG_ERROR,"Error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());     
        }
      }  
    }  
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

bool SMTP_Connect(){
  Debug.print(DBG_DEBUG,"SMTP Connection starts");
  if (!smtp.connect(&config)){
    Debug.print(DBG_ERROR,"SMTP Connection error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return false;
  } else Debug.print(DBG_INFO,"SMTP Connected");
  if (!smtp.isLoggedIn()) Debug.print(DBG_ERROR,"Not yet logged in.");
  else{
    if (smtp.isAuthenticated()) Debug.print(DBG_INFO,"SMTP Successfully logged in.");
    else Debug.print(DBG_ERROR,"SMTP Connected with no Auth.");
  }
  return true;
}

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status){
  /* Print the current status */
  Debug.print(DBG_INFO,"Email send status: %d",status.info());

  /* Print the sending result */
  if (status.success()){
    Debug.print(DBG_INFO,"Message sent success: %d", status.completedCount());
    Debug.print(DBG_INFO,"Message sent failed: %d", status.failedCount());

    for (size_t i = 0; i < smtp.sendingResult.size(); i++)
    {
      /* Get the result item */
      SMTP_Result result = smtp.sendingResult.getItem(i);

      // In case, ESP32, ESP8266 and SAMD device, the timestamp get from result.timestamp should be valid if
      // your device time was synched with NTP server.
      // Other devices may show invalid timestamp as the device time was not set i.e. it will show Jan 1, 1970.
      // You can call smtp.setSystemTime(xxx) to set device time manually. Where xxx is timestamp (seconds since Jan 1, 1970)
      
      Debug.print(DBG_INFO,"Message No: %d", i + 1);
      Debug.print(DBG_INFO,"Status: %s", result.completed ? "success" : "failed");
      Debug.print(DBG_INFO,"Date/Time: %s", MailClient.Time.getDateTimeString(result.timestamp, "%B %d, %Y %H:%M:%S").c_str());
      Debug.print(DBG_INFO,"Recipient: %s", result.recipients.c_str());
      Debug.print(DBG_INFO,"Subject: %s", result.subject.c_str());
    }

    // Clear sending result as the memory usage will grow up.
    smtp.sendingResult.clear();
  }
}