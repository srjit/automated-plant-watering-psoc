/******************************************************************************
* File Name: html_web_page.h
*
* Description: This file contains the HTML pages that the server will host and
*              macros required for http transaction.
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Include guard
*******************************************************************************/
#ifndef HTML_WEB_PAGE_H_
#define HTML_WEB_PAGE_H_

/*******************************************************************************
* Macros
******************************************************************************/
/* Company Logo */
#define LOGO \
    "<style>" \
        ".container { " \
            "position: relative; " \
        "}" \
        ".topleft{" \
            "position: absolute; " \
            "top: 8px; " \
            "left: 16px; "\
            "font-size: 18px; " \
        "}" \
        "img {" \
            "width : auto; " \
            "height: auto;" \
        "}" \
    "</style>" \
    "<div class=\"container\"> "\
    "<img alt=\"logo.png\" "\

"


/* HTML Device Data Page - Data  */
#define SOFTAP_DEVICE_DATA \
        "<!DOCTYPE html> " \
        "<html>" \
        "<head><title>Wi-Fi Web Server Demo Device Status</title></head>" \
        "<body>" \
            "<h1 style=\"text-align: center\" > H2Ohms! </h1>" LOGO \
            "<br><br>" \
            "<p>Click to turn on a Pump</p>" \
			"<button type=\"button\" onclick=\"pump1()\" id=\"pump1_btn\">Pump1</button> " \
			"<button type=\"button\" onclick=\"pump2()\" id=\"pump2_btn\">Pump2</button> " \
			"<button type=\"button\" onclick=\"pump3()\" id=\"pump3_btn\">Pump3</button> " \
			"<button type=\"button\" onclick=\"pump4()\" id=\"pump4_btn\">Pump4</button> " \
            "<br><br>" \
            "<br><br>" \
            "<div id=\"device_data\" value=\"100\"></div>" \
            "<script>" \
                " function btn_disable_function() {" \
                " var pump1_btn_id = document.getElementById(\"pump1_btn\");" \
                " var pump2_btn_id = document.getElementById(\"pump2_btn\");" \
                " var pump3_btn_id = document.getElementById(\"pump3_btn\");" \
                " var pump4_btn_id = document.getElementById(\"pump4_btn\");" \
                " pump1_btn_id.innerText = \"Please Wait...\";" \
                " pump2_btn_id.innerText = \"Please Wait...\";" \
                " pump3_btn_id.innerText = \"Please Wait...\";" \
                " pump4_btn_id.innerText = \"Please Wait...\";" \
                " pump1_btn_id.disabled = true;" \
                " pump2_btn_id.disabled = true;" \
                " pump3_btn_id.disabled = true;" \
                " pump4_btn_id.disabled = true;" \
                " setTimeout(function()" \
                " {" \
                    " pump1_btn_id.innerText = \"Pump1\";" \
                    " pump2_btn_id.innerText = \"Pump2\";" \
                    " pump3_btn_id.innerText = \"Pump3\";" \
                    " pump4_btn_id.innerText = \"Pump4\";" \
                    " pump1_btn_id.disabled = false;" \
                    " pump2_btn_id.disabled = false;" \
                    " pump3_btn_id.disabled = false;" \
                    " pump4_btn_id.disabled = false;" \
                    " },1000);" \
                " }" \
			"function pump1() { " \
                "  btn_disable_function();" \
                "  var xhttp = new XMLHttpRequest(); " \
                "  xhttp.onreadystatechange = function() { " \
                    "    if (this.readyState === 4 && this.status == 200) { " \
                    "    } " \
                    "  }; " \
                    "xhttp.open(\"POST\", \"/\", true); " \
                    "xhttp.setRequestHeader(\"Content-type\", \"application/x-www-form-urlencoded\"); "\
                    "xhttp.send(\"Pump1\"); " \
            "} "\
			"function pump2() { " \
                "  btn_disable_function();" \
                "  var xhttp = new XMLHttpRequest(); " \
                "  xhttp.onreadystatechange = function() { " \
                    "    if (this.readyState === 4 && this.status == 200) { " \
                    "    } " \
                    "  }; " \
                    "xhttp.open(\"POST\", \"/\", true); " \
                    "xhttp.setRequestHeader(\"Content-type\", \"application/x-www-form-urlencoded\"); "\
                    "xhttp.send(\"Pump2\"); " \
            "} "\
			"function pump3() { " \
                "  btn_disable_function();" \
                "  var xhttp = new XMLHttpRequest(); " \
                "  xhttp.onreadystatechange = function() { " \
                    "    if (this.readyState === 4 && this.status == 200) { " \
                    "    } " \
                    "  }; " \
                    "xhttp.open(\"POST\", \"/\", true); " \
                    "xhttp.setRequestHeader(\"Content-type\", \"application/x-www-form-urlencoded\"); "\
                    "xhttp.send(\"Pump3\"); " \
            "} "\
			"function pump4() { " \
                "  btn_disable_function();" \
                "  var xhttp = new XMLHttpRequest(); " \
                "  xhttp.onreadystatechange = function() { " \
                    "    if (this.readyState === 4 && this.status == 200) { " \
                    "    } " \
                    "  }; " \
                    "xhttp.open(\"POST\", \"/\", true); " \
                    "xhttp.setRequestHeader(\"Content-type\", \"application/x-www-form-urlencoded\"); "\
                    "xhttp.send(\"Pump4\"); " \
            "} "\
        "if(typeof(EventSource) !== \"undefined\") {" \
            "var source = new EventSource(\"/events\");" \
            "source.onmessage = function(event) {" \
                "document.getElementById(\"device_data\").innerHTML = event.data;" \
                "  };" \
        "} else {" \
            "document.getElementById(\"device_data\").innerHTML = \"Sorry, your browser does not support server-sent events...\";" \
        "}" \
        "</script>" \
        "</body>" \
        "</html>"

#endif /* HTTP_PAGES_H_ */

/* [] END OF FILE */
