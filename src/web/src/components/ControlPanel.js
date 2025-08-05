import React, { useState } from 'react';
import { Card, Slider, Button, Space, Typography, message } from 'antd';
import { SettingOutlined, SaveOutlined } from '@ant-design/icons';

const { Title, Text } = Typography;

const ControlPanel = ({ targetHumidity, onTargetHumidityChange, loading }) => {
  const [localTarget, setLocalTarget] = useState(targetHumidity);

  const handleSave = () => {
    if (localTarget !== targetHumidity) {
      onTargetHumidityChange(localTarget);
    } else {
      message.info('目标湿度未发生变化');
    }
  };

  const handleReset = () => {
    setLocalTarget(targetHumidity);
  };

  const marks = {
    30: '30%',
    40: '40%',
    50: '50%',
    60: '60%',
    70: '70%',
    80: '80%'
  };

  return (
    <Card>
      <Title level={4} style={{ marginBottom: '20px' }}>
        <SettingOutlined /> 湿度控制设置
      </Title>
      
      <div style={{ marginBottom: '24px' }}>
        <Text strong>目标湿度: {localTarget}%</Text>
        <Slider
          min={30}
          max={80}
          value={localTarget}
          onChange={setLocalTarget}
          marks={marks}
          tooltip={{
            formatter: (value) => `${value}%`
          }}
          style={{ marginTop: '16px' }}
        />
      </div>

      <div style={{ marginBottom: '16px' }}>
        <Text type="secondary">
          建议湿度范围: 40% - 70%
        </Text>
      </div>

      <Space>
        <Button 
          type="primary" 
          icon={<SaveOutlined />}
          onClick={handleSave}
          loading={loading}
        >
          保存设置
        </Button>
        <Button onClick={handleReset}>
          重置
        </Button>
      </Space>

      <div style={{ marginTop: '16px', padding: '12px', background: '#f6f8fa', borderRadius: '6px' }}>
        <Text type="secondary" style={{ fontSize: '12px' }}>
          <strong>说明:</strong> 系统会根据目标湿度自动控制加湿器或除湿器。
          当实际湿度低于目标湿度2%时启动加湿，高于目标湿度2%时启动除湿。
        </Text>
      </div>
    </Card>
  );
};

export default ControlPanel; 