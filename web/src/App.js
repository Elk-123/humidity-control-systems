import React, { useState, useEffect } from 'react';
import { Layout, Typography, Space, message } from 'antd';
import { 
  CloudOutlined, 
  ThermometerOutlined, 
  SettingOutlined,
  PoweroffOutlined,
  PoweroffOutlined as PowerOnOutlined
} from '@ant-design/icons';
import SensorDisplay from './components/SensorDisplay';
import ControlPanel from './components/ControlPanel';
import DeviceStatus from './components/DeviceStatus';
import DataChart from './components/DataChart';
import { fetchSensorData, updateTargetHumidity, controlDevice } from './services/api';
import './App.css';

const { Header, Content } = Layout;
const { Title } = Typography;

function App() {
  const [sensorData, setSensorData] = useState({
    temperature: 0,
    humidity: 0,
    targetHumidity: 60,
    timestamp: new Date()
  });
  
  const [deviceStatus, setDeviceStatus] = useState({
    humidifier: false,
    dehumidifier: false,
    systemOnline: false
  });
  
  const [loading, setLoading] = useState(false);
  const [historicalData, setHistoricalData] = useState([]);

  // 模拟数据更新（实际项目中会从API获取）
  useEffect(() => {
    const interval = setInterval(() => {
      // 模拟传感器数据
      const mockData = {
        temperature: 20 + Math.random() * 10,
        humidity: 40 + Math.random() * 30,
        targetHumidity: sensorData.targetHumidity,
        timestamp: new Date()
      };
      
      setSensorData(mockData);
      
      // 更新历史数据
      setHistoricalData(prev => {
        const newData = [...prev, {
          time: mockData.timestamp,
          temperature: mockData.temperature,
          humidity: mockData.humidity
        }];
        
        // 只保留最近100个数据点
        return newData.slice(-100);
      });
      
      // 模拟设备状态
      setDeviceStatus(prev => ({
        ...prev,
        systemOnline: true,
        humidifier: mockData.humidity < mockData.targetHumidity - 2,
        dehumidifier: mockData.humidity > mockData.targetHumidity + 2
      }));
    }, 2000);

    return () => clearInterval(interval);
  }, [sensorData.targetHumidity]);

  const handleTargetHumidityChange = async (newTarget) => {
    try {
      setLoading(true);
      // 实际项目中会调用API
      // await updateTargetHumidity(newTarget);
      
      setSensorData(prev => ({
        ...prev,
        targetHumidity: newTarget
      }));
      
      message.success(`目标湿度已设置为 ${newTarget}%`);
    } catch (error) {
      message.error('设置目标湿度失败');
    } finally {
      setLoading(false);
    }
  };

  const handleDeviceControl = async (device, action) => {
    try {
      setLoading(true);
      // 实际项目中会调用API
      // await controlDevice(device, action);
      
      setDeviceStatus(prev => ({
        ...prev,
        [device]: action === 'on'
      }));
      
      message.success(`${device === 'humidifier' ? '加湿器' : '除湿器'}已${action === 'on' ? '开启' : '关闭'}`);
    } catch (error) {
      message.error('设备控制失败');
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout className="humidity-control-app">
      <Header style={{ 
        background: 'rgba(255, 255, 255, 0.1)', 
        backdropFilter: 'blur(10px)',
        borderBottom: '1px solid rgba(255, 255, 255, 0.2)'
      }}>
        <div style={{ display: 'flex', alignItems: 'center', height: '100%' }}>
          <CloudOutlined style={{ fontSize: '24px', color: 'white', marginRight: '12px' }} />
          <Title level={3} style={{ color: 'white', margin: 0 }}>
            湿度控制系统
          </Title>
        </div>
      </Header>
      
      <Content className="main-container">
        <Space direction="vertical" size="large" style={{ width: '100%' }}>
          {/* 传感器数据显示 */}
          <div className="status-card">
            <Title level={4} style={{ marginBottom: '20px' }}>
              <ThermometerOutlined /> 实时环境数据
            </Title>
            <SensorDisplay 
              temperature={sensorData.temperature}
              humidity={sensorData.humidity}
              targetHumidity={sensorData.targetHumidity}
              timestamp={sensorData.timestamp}
            />
          </div>

          {/* 控制面板 */}
          <div className="control-panel">
            <ControlPanel
              targetHumidity={sensorData.targetHumidity}
              onTargetHumidityChange={handleTargetHumidityChange}
              loading={loading}
            />
            
            <DeviceStatus
              deviceStatus={deviceStatus}
              onDeviceControl={handleDeviceControl}
              loading={loading}
            />
          </div>

          {/* 数据图表 */}
          <div className="chart-container">
            <Title level={4} style={{ marginBottom: '20px' }}>
              <SettingOutlined /> 历史数据趋势
            </Title>
            <DataChart data={historicalData} />
          </div>
        </Space>
      </Content>
    </Layout>
  );
}

export default App; 