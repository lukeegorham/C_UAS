-----------------------------
     UAV Mavlink Object
-----------------------------

VARIABLES:
- mavConnection ~ Mavlink connection with UAV (complex object)
- allowGsControl ~ boolean allowing groundstation to control UAV
- normalControl ~ boolean allowing ___???___ to control UAV (assume with std controller)
- portNotConnected ~ _________???_____________
- udpSvrSocketPort
- udpRcvSocketPort
- bufferSize ~ size of buffer (bits) for reading in message???
- home_alt_mMSL ~ home altitude in meters MSL
- home_lat_deg
- home_lon_deg
- currentLat_Deg
- currentLon_Deg
- currentAlt_mMSL
- currentVelX_mps
- currentVelY_mps
- currentVelZ_mps
- currentHeading_deg
- lat_degE7 ~ ???
- lon_degE7 ~ ???
- alt_mmMSL ~ ???
- velX_cmps ~ ???
- velY_cmps ~ ???
- velZ_cmps ~ ???
- heading_cdeg ~ ???

FUNCTIONS:
- Wait Heartbeat ~ call to mavConnection function waiting to come online
- Read Mavlink and Forward Udp ~ read message(???) and forward via UDP to (???)
- Read UDP and Send Mavlink Command ~ takes UDP message and sends via Mavlink mavConnection
- Return to Land
- Fly To Waypoint ~ flies to lat/long/alt
- Set Gimbal RC ~ _________???__________
- Set ROI ~ _________???__________
- Request Home Position ~ gets lat/long/alt of home from mavConnection
- Send Tilt ~ sends tilt command to UAV via Mavlink (what purpose???)
- Close Mavlink
- Init Mavlink