import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';

class BluetoothOffScreen extends StatelessWidget {
  const BluetoothOffScreen({Key? key, this.state}) : super(key: key);

  final BluetoothAdapterState? state;

  @override 
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.red,
      body: Center( 
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            const Icon(
              Icons.bluetooth_disabled,
              size: 200.0,
              color: Colors.white,
            ),
            const SizedBox(height: 12),
            Text(
              'Bluetooth Adapter is ${state != null ? state.toString().substring(15) : 'not available'}.',
              style: Theme.of(context)
              .primaryTextTheme
              .titleSmall
              ?.copyWith(color: Colors.white),
            ),
            const SizedBox(height: 12),
            ElevatedButton(
              style: ElevatedButton.styleFrom(
                foregroundColor: Colors.red,
                backgroundColor: Colors.white,
              ),
              onPressed: Platform.isAndroid
              ? () async {
                if(!await Permission.bluetoothConnect.isGranted) {
                  await Permission.bluetoothConnect.request();
                }
                FlutterBluePlus.turnOn();
              }
              : null,
              child: const Text('Turn On'),
            ),
          ],
        ),
      ),
    );
  }
}