// import 'dart:html';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';

class BluetoothOnScreen extends StatelessWidget {
  const BluetoothOnScreen({Key? key, this.state}) : super(key:key);

  final BluetoothAdapterState? state;

  @override 
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.blue,
      appBar: AppBar(
        elevation: 0,
        title: const Text('Find Devices'),
        actions: [
          ElevatedButton(
            style: ElevatedButton.styleFrom(
              foregroundColor: Colors.blue,
              backgroundColor: Colors.white,
            ),
            onPressed: Platform.isAndroid 
              ? () async {
                if (await Permission.bluetoothConnect.isGranted) {
                   FlutterBluePlus.turnOff();
                } else {
                  PermissionStatus permissionStatus = await Permission.bluetoothConnect.request();
                  if(permissionStatus == PermissionStatus.granted) {
                    FlutterBluePlus.turnOff();
                  }
                }
              }
              : null,
            child: const Text('Turn Off'),
          )
        ],
      ),
      body: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            const Icon(
              Icons.bluetooth_searching,
              size: 200.0,
              color: Colors.white,
            ),
            const SizedBox(height: 12),
            Text(
              'Bluetooth Adapter is ${state !=null ? state.toString().substring(15) : 'not available'}.',
              style: Theme.of(context)
                .primaryTextTheme
                .titleSmall
                ?.copyWith(color: Colors.white),
            )
          ],
        ),
      )
    );
  }
}