import React from 'react';
import { Card, Switch, Button, Space, Typography, Tag } from 'antd';
import { 
  PoweroffOutlined, 
  CloudOutlined, 
  ThunderboltOutlined,
  WifiOutlined,
  WifiOutlined as WifiOffOutlined
} from '@ant-design/icons';

const { Title, Text } = Typography;

const DeviceStatus = ({ deviceStatus, onDeviceControl, loading }) => {
  const { humidifier, dehumidifier, systemOnline } = deviceStatus;

  const handleDeviceToggle = (device) => {
    const newState = !deviceStatus[device];
    onDeviceControl(device, newState ? 'on' : 'off');
  };

  const getDeviceIcon = (device) => {
    return device === 'humidifier' ? <CloudOutlined /> : <ThunderboltOutlined />;
  };

  const getDeviceName = (device) => {
    return device === 'humidifier' ? '加湿器' : '除湿器';
  };

  const getDeviceColor = (device, isOn) => {
    if (!isOn) return '#d9d9d9';
    return device === 'humidifier' ? '#52c41a' : '#1890ff';
  };

  return (
    <Card>
      <Title level={4} style={{ marginBottom: '20px' }}>
        <PoweroffOutlined /> 设备状态控制
      </Title>

      {/* 系统状态 */}
      <div style={{ marginBottom: '20px', padding: '12px', background: '#f6f8fa', borderRadius: '6px' }}>
        <Space>
          {systemOnline ? (
            <WifiOutlined style={{ color: '#52c41a' }} />
          ) : (
            <WifiOffOutlined style={{ color: '#ff4d4f' }} />
          )}
          <Text strong>系统状态:</Text>
          <Tag color={systemOnline ? 'green' : 'red'}>
            {systemOnline ? '在线' : '离线'}
          </Tag>
        </Space>
      </div>

      {/* 设备控制 */}
      <div style={{ marginBottom: '16px' }}>
        <div className="device-status">
          <div className="device-name">
            <Space>
              {getDeviceIcon('humidifier')}
              加湿器
            </Space>
          </div>
          <div className="device-state">
            <Space>
              <Text type="secondary">
                {humidifier ? '运行中' : '已停止'}
              </Text>
              <Switch
                checked={humidifier}
                onChange={() => handleDeviceToggle('humidifier')}
                disabled={loading || !systemOnline}
                style={{ backgroundColor: getDeviceColor('humidifier', humidifier) }}
              />
            </Space>
          </div>
        </div>

        <div className="device-status">
          <div className="device-name">
            <Space>
              {getDeviceIcon('dehumidifier')}
              除湿器
            </Space>
          </div>
          <div className="device-state">
            <Space>
              <Text type="secondary">
                {dehumidifier ? '运行中' : '已停止'}
              </Text>
              <Switch
                checked={dehumidifier}
                onChange={() => handleDeviceToggle('dehumidifier')}
                disabled={loading || !systemOnline}
                style={{ backgroundColor: getDeviceColor('dehumidifier', dehumidifier) }}
              />
            </Space>
          </div>
        </div>
      </div>

      {/* 快速控制按钮 */}
      <div style={{ marginTop: '16px' }}>
        <Text strong style={{ display: 'block', marginBottom: '8px' }}>
          快速控制:
        </Text>
        <Space wrap>
          <Button
            type="primary"
            size="small"
            onClick={() => onDeviceControl('humidifier', 'on')}
            disabled={loading || !systemOnline || humidifier}
            icon={<CloudOutlined />}
          >
            开启加湿
          </Button>
          <Button
            size="small"
            onClick={() => onDeviceControl('humidifier', 'off')}
            disabled={loading || !systemOnline || !humidifier}
          >
            关闭加湿
          </Button>
          <Button
            type="primary"
            size="small"
            onClick={() => onDeviceControl('dehumidifier', 'on')}
            disabled={loading || !systemOnline || dehumidifier}
            icon={<ThunderboltOutlined />}
          >
            开启除湿
          </Button>
          <Button
            size="small"
            onClick={() => onDeviceControl('dehumidifier', 'off')}
            disabled={loading || !systemOnline || !dehumidifier}
          >
            关闭除湿
          </Button>
        </Space>
      </div>

      <div style={{ marginTop: '16px', padding: '12px', background: '#fff7e6', borderRadius: '6px' }}>
        <Text type="secondary" style={{ fontSize: '12px' }}>
          <strong>注意:</strong> 加湿器和除湿器不会同时工作，系统会自动协调控制。
        </Text>
      </div>
    </Card>
  );
};

export default DeviceStatus; 