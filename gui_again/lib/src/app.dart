import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:flutter_localizations/flutter_localizations.dart';

// import 'sample_feature/sample_item_details_view.dart';
// import 'sample_feature/sample_item_list_view.dart';
// import 'settings/settings_controller.dart';
// import 'settings/settings_view.dart';

import 'screens/ble_connected_screen.dart';
import 'screens/ble_disconnected_screen.dart';

/// The Widget that configures your application.
class MyApp extends StatelessWidget {
  const MyApp({
    super.key,
    // required this.settingsController,
    required this.bluetoothOn,
    required this.bluetoothOff,
  });

  final BluetoothOffScreen bluetoothOff;
  final BluetoothOnScreen bluetoothOn;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'TrailSense GUI',
      theme: ThemeData(primarySwatch: Colors.blue),
      color: Colors.blue,
      home: StreamBuilder<BluetoothAdapterState>(
        stream: FlutterBluePlus.adapterState,
        initialData: BluetoothAdapterState.unknown,
        builder: (c, snapshot) {
          final state = snapshot.data;
          if(state == BluetoothAdapterState.on) {
            return BluetoothOnScreen(state: state);
          }
          return BluetoothOffScreen(state: state);
        }
      ),
    );
  }
}
